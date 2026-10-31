// lhsyncd's Galaxy XR side (src/headset/headset.h): Android properties, Google's OpenPX camera calibration, and
// lhsight-gxr (gxrsight.c), the camera reader: a QVR camera client of its own, as Samsung's controller service is one.
// The Galaxy XR uses Samsung S5K931 global-shutter tracking cameras (640x640 Y8), managed by Google's Perception stack
// through android.hardware.perceptionhost instead of Meta's sensors HAL.
#include <dirent.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/system_properties.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <android/log.h>

#include "headset.h"


// Google's OpenPX calibration: the device profile, and this unit's refinements of some cameras' extrinsics. (The QVR
// XML under /data/vendor/qvr is an unconfigured default: its rig is 15 deg off what the cameras see.)
#define OPX_PROFILE "/vendor/etc/calibration_service/device_profile.textproto"
#define OPX_RECORDS "/data/vendor/openpx/calibration/records"

const char *const hs_capture_name = "lhsight-gxr";
static const char *moddir = "/data/adb/modules/questlhsync";

void hs_log(const char *line) { __android_log_print(ANDROID_LOG_INFO, "QuestLHSync", "%s", line); }

void hs_device_info(char *serial, char *model, char *fw, size_t n) {
  (void)n;
  __system_property_get("ro.serialno", serial);
  __system_property_get("ro.product.model", model);
  __system_property_get("ro.build.version.incremental", fw);
  if (!model[0]) strcpy(model, "Galaxy_XR");
}

static char *read_file(const char *path, size_t *len) {
  FILE *f = fopen(path, "rb");
  if (!f) return NULL;
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  fseek(f, 0, SEEK_SET);
  char *buf = malloc(n > 0 ? n + 1 : 1);
  size_t got = n > 0 ? fread(buf, 1, n, f) : 0;
  fclose(f);
  buf[got] = 0;
  *len = got;
  return buf;
}

// ---------------------------------------------------------------- calibration (OpenPX textproto)
// The tracking cameras lhsight reads, by the ids it sends them as. trackingA/B sit low and look 25 deg down,
// trackingC/D sit high and look level; each pair 23 deg out to either side. This unit runs the _curved profiles (its
// own records refine those).
static const char *const kCam[] = {"trackingA_curved", "trackingB_curved", "trackingC_curved", "trackingD_curved"};
#define NCAM ((int)(sizeof kCam / sizeof kCam[0]))
// The HMD pose's origin in IMU_0 (OpenGL axes: x right, y up, z back). First guessed from the eye cameras' places
// (between the eyes, 2.5 cm of eye relief behind the lenses: 0.0625, -0.035, 0.050); measured with a tracker on a
// headset mount (5.8 min, 680 station sightings): 5.6 cm further back, 1.6 cm lower, 1 cm to the right
// (+-2 cm), which took the solve from 13 cm off the tracker's truth to 4
static const double kEye[3] = {0.0725, -0.051, 0.106};

// the body of a "key {" block at or after s (before e), braces matched
static int opx_sub(const char *s, const char *e, const char *key, const char **bs, const char **be) {
  size_t kl = strlen(key);
  for (const char *p = s; p && p < e; p++) {
    p = strstr(p, key);
    if (!p || p + kl > e) return 0;
    const char *q = p + kl;
    while (q < e && (*q == ' ' || *q == '\t')) q++;
    if (q >= e || *q != '{') continue;
    int depth = 0;
    for (const char *r = q; r < e; r++) {
      if (*r == '{') depth++;
      else if (*r == '}' && --depth == 0) { *bs = q + 1; *be = r; return 1; }
    }
    return 0;
  }
  return 0;
}

// a top-level "name {" block naming camera id
static int opx_block(const char *t, const char *e, const char *name, const char *id, const char **bs, const char **be) {
  char want[96];
  snprintf(want, sizeof want, "camera_id: \"%s\"", id);
  size_t nl = strlen(name);
  for (const char *p = t; p < e; p++) {
    if (p != t && p[-1] != '\n') continue;
    if (strncmp(p, name, nl) || p[nl] != ' ') continue;
    const char *s0, *e0;
    if (!opx_sub(p, e, name, &s0, &e0)) return 0;
    const char *c = strstr(s0, want);
    if (c && c < e0) { *bs = s0; *be = e0; return 1; }
    p = e0;
  }
  return 0;
}

// "field: value" directly in [s, e) (protobuf leaves zeros out: missing reads as 0)
static double opx_num(const char *s, const char *e, const char *field) {
  char want[64];
  snprintf(want, sizeof want, "%s:", field);
  size_t wl = strlen(want);
  for (const char *p = s; p && p < e; p++) {
    p = strstr(p, want);
    if (!p || p >= e) break;
    if (p != s && (p[-1] == '_' || (p[-1] >= 'a' && p[-1] <= 'z'))) continue;  // "x:" inside "max:"
    return strtod(p + wl, NULL);
  }
  return 0;
}

static int opx_vec(const char *s, const char *e, const char *key, const char *const *f, int n, double *out) {
  const char *bs, *be;
  if (!opx_sub(s, e, key, &bs, &be)) return 0;
  for (int i = 0; i < n; i++) out[i] = opx_num(bs, be, f[i]);
  return 1;
}

// a pose block (frame_t_camera, a_t_b): p, and the rotation camera -> IMU_0. OpenPX writes JPL quaternions: as
// Hamilton ones they turn the other way (the cameras' measured relative rotations say so), hence the transpose.
static int opx_pose(const char *s, const char *e, const char *key, double t[3], double R[9]) {
  static const char *const xyz[] = {"x", "y", "z"}, *const wxyz[] = {"w", "x", "y", "z"};
  const char *bs, *be;
  double q[4];
  if (!opx_sub(s, e, key, &bs, &be) || !opx_vec(bs, be, "p", xyz, 3, t) || !opx_vec(bs, be, "q", wxyz, 4, q)) return 0;
  double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (!(n > 0.5)) return 0;
  double w = q[0] / n, x = q[1] / n, y = q[2] / n, z = q[3] / n;
  double H[9] = {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
                 2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
                 2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)};
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) R[r * 3 + c] = H[c * 3 + r];
  return 1;
}

int hs_calibration(char **data, size_t *len, char *name, size_t n) {
  // a calibration refined elsewhere (fitted to recorded base station sightings) wins over the profile's
  char op[512];
  snprintf(op, sizeof op, "%s/calibration_override.json", moddir);
  size_t olen;
  char *ov = read_file(op, &olen);
  if (ov && olen) {
    *data = ov;
    *len = olen;
    snprintf(name, n, "override %s", op);
    return 0;
  }
  free(ov);
  size_t plen;
  char *prof = read_file(OPX_PROFILE, &plen);
  if (!prof || !plen) {
    free(prof);
    snprintf(name, n, "can't read %s", OPX_PROFILE);
    return -1;
  }
  const char *pe = prof + plen;
  size_t jcap = 8192, jo = 0;
  char *json = malloc(jcap);
  #define JP(...) do { jo += snprintf(json + jo, jcap - jo, __VA_ARGS__); } while (0)
  // galaxy_xr_sensor: lhsight's frame times are the sensors' own (mid-exposure, CLOCK_MONOTONIC), not when it found them
  JP("{\"kind\": \"galaxy_xr_sensor\", \"GxrCameraCalibration\": [");
  int ncam = 0, nrec = 0;
  char missing[256] = "";
  for (int i = 0; i < NCAM; i++) {
    const char *cb, *ce, *xb, *xe;
    double t[3], R[9];
    if (!opx_block(prof, pe, "cameras", kCam[i], &cb, &ce) || !opx_block(prof, pe, "camera_extrinsics", kCam[i], &xb, &xe) ||
        !opx_pose(xb, xe, "frame_t_camera", t, R)) {
      snprintf(missing + strlen(missing), sizeof missing - strlen(missing), " %s", kCam[i]);
      continue;
    }
    char rp[256];
    snprintf(rp, sizeof rp, "%s/FRAME_IMU_0_t_CAMERA_%s_.textproto", OPX_RECORDS, kCam[i]);
    size_t rlen;
    char *rec = read_file(rp, &rlen);
    if (rec && rlen && opx_pose(rec, rec + rlen, "a_t_b", t, R)) nrec++;
    free(rec);
    static const char *const xy[] = {"x", "y"};
    double f[2], c[2];
    if (!opx_vec(cb, ce, "calibrated_focal_length", xy, 2, f) || !opx_vec(cb, ce, "calibrated_principal_point", xy, 2, c) ||
        !(f[0] > 0 && f[1] > 0)) {
      snprintf(missing + strlen(missing), sizeof missing - strlen(missing), " %s(intrinsics)", kCam[i]);
      continue;
    }
    double w = opx_num(cb, ce, "calibrated_width"), h = opx_num(cb, ce, "calibrated_height");
    if (ncam) JP(",");
    JP("\n  {\"Id\": %d, \"Name\": \"%s\", \"ImageSize\": [%.0f, %.0f],", i, kCam[i], w > 0 ? w : 640, h > 0 ? h : 640);
    // DeviceFromCamera: 4x4 row-major [R | t; 0 0 0 1], device = the HMD pose's frame (IMU_0's axes, the eyes' origin)
    JP(" \"DeviceFromCamera\": [%.10g, %.10g, %.10g, %.10g, %.10g, %.10g, %.10g, %.10g, %.10g, %.10g, %.10g, %.10g, 0, 0, 0, 1],",
       R[0], R[1], R[2], t[0] - kEye[0], R[3], R[4], R[5], t[1] - kEye[1], R[6], R[7], R[8], t[2] - kEye[2]);
    JP(" \"Projection\": {\"Model\": \"PinholeXY\", \"Coefficients\": [%.10g, %.10g, %.10g, %.10g]},", f[0], f[1], c[0], c[1]);
    // distortion_fisheye_spline3 isn't a KB polynomial (read as one it runs away at the rim); the lenses are near
    // ideal equidistant fisheyes, which is what all-zero coefficients give
    JP(" \"Distortion\": {\"Coefficients\": [0, 0, 0, 0, 0, 0, 0, 0]},");
    JP(" \"SensorType\": \"S5K931\"}");
    ncam++;
  }
  JP("\n]}\n");
  #undef JP
  free(prof);
  if (!ncam) {
    free(json);
    snprintf(name, n, "no tracking cameras in %s:%s", OPX_PROFILE, missing);
    return -1;
  }
  *data = json;
  *len = jo;
  snprintf(name, n, "openpx/device_profile.textproto (%d cameras, %d with this unit's records)%s%s", ncam, nrec,
           missing[0] ? "; missing:" : "", missing);
  return 0;
}

// ---------------------------------------------------------------- lhsight-gxr
// a stray reader (this daemon SIGKILLed, an old adb session) would keep its camera clients and the frame pattern:
// lhsight-gxr, or the Frida script inside perceptionhost that earlier modules ran
static int is_lhsight(const char *pid) {
  char p[64], cmd[1024];
  snprintf(p, sizeof p, "/proc/%s/cmdline", pid);
  int fd = open(p, O_RDONLY);
  if (fd < 0) return 0;
  ssize_t len = read(fd, cmd, sizeof cmd - 1);
  close(fd);
  if (len <= 0) return 0;
  cmd[len] = 0;
  const char *base = strrchr(cmd, '/') ? strrchr(cmd, '/') + 1 : cmd;
  if (!strcmp(base, "lhsight-gxr")) return 1;
  if (strcmp(base, "frida-inject")) return 0;
  for (ssize_t i = strlen(cmd) + 1; i < len; i += strlen(cmd + i) + 1) {
    size_t l = strlen(cmd + i);
    if (l >= 10 && !strcmp(cmd + i + l - 10, "lhsight.js")) return 1;
  }
  return 0;
}

static void kill_strays(void) {
  DIR *d = opendir("/proc");
  if (!d) return;
  struct dirent *e;
  char found[16][16];
  int n = 0;
  while (n < 16 && (e = readdir(d)))
    if (e->d_name[0] >= '0' && e->d_name[0] <= '9' && is_lhsight(e->d_name)) snprintf(found[n++], 16, "%s", e->d_name);
  closedir(d);
  for (int i = 0; i < n; i++) {
    char m[96];
    snprintf(m, sizeof m, "stopping a stray lhsight (pid %s)", found[i]);
    hs_log(m);
    kill(atoi(found[i]), SIGTERM);
  }
  for (int t = 0; n && t < 30; t++) {
    usleep(100000);
    int left = 0;
    for (int i = 0; i < n; i++) left += is_lhsight(found[i]);
    if (!left) return;
  }
  for (int i = 0; i < n; i++)
    if (is_lhsight(found[i])) kill(atoi(found[i]), SIGKILL);
}

void hs_init(int argc, char **argv) {
  if (argc > 1) moddir = argv[1];
  kill_strays();
  char m[600];
  snprintf(m, sizeof m, "module %s (Galaxy XR)", moddir);
  hs_log(m);
}

// The linker lets an executable under /data (the module) load no vendor library: /linkerconfig/ld.config.txt files
// /data under "system", whose namespace leaves /vendor/lib64 out, and libqvrcamera_client.qti.so is one. /data/local/tmp
// is "unrestricted": lhsight-gxr runs from a copy there
#define RUN_EXE "/data/local/tmp/questlhsync/lhsight-gxr"
static int copy_file(const char *src, const char *dst) {
  size_t len;
  char *data = read_file(src, &len);
  if (!data) return -1;
  char tmp[512];
  snprintf(tmp, sizeof tmp, "%s.new", dst);
  FILE *f = fopen(tmp, "wb");
  int ok = f && fwrite(data, 1, len, f) == len;
  if (f) fclose(f);
  free(data);
  if (!ok || chmod(tmp, 0755) || rename(tmp, dst)) { unlink(tmp); return -1; }
  return 0;
}

// lhsight-gxr with the ids hs_calibration gives trackingA..D
pid_t hs_capture_start(int out_fd, char *msg, size_t n) {
  char src[512], id[NCAM][8];
  const char *exe = RUN_EXE;
  snprintf(src, sizeof src, "%s/lhsight-gxr", moddir);
  if (access(src, R_OK)) { snprintf(msg, n, "no %s", src); return -1; }
  mkdir("/data/local/tmp/questlhsync", 0755);
  if (copy_file(src, exe)) { snprintf(msg, n, "can't copy %s to %s", src, exe); return -1; }
  for (int i = 0; i < NCAM; i++) snprintf(id[i], sizeof id[i], "%d", i);
  pid_t pid = fork();
  if (pid == 0) {
    setsid();
    dup2(out_fd, 1);
    dup2(out_fd, 2);
    int nul = open("/dev/null", O_RDONLY);
    dup2(nul, 0);
    for (int fd = 3; fd < 1024; fd++) close(fd);
    execl(exe, "lhsight-gxr", id[0], id[1], id[2], id[3], (char *)NULL);
    _exit(127);
  }
  if (pid < 0) return -1;
  snprintf(msg, n, "lhsight-gxr started (pid %d): its own QVR camera clients, cameras %s %s %s %s as 0-3", pid, kCam[0],
           kCam[1], kCam[2], kCam[3]);
  return pid;
}

