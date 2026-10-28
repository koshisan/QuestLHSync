// lhsyncd's Galaxy XR side (src/headset/headset.h): Android properties, Qualcomm QVR camera calibration, and lhsight
// as a Frida script (frida-inject + lhsight.js) inside perceptionhost: it reads the camera rings and writes nothing.
// The Galaxy XR uses Samsung S5K931 global-shutter tracking cameras (640x640 Y8), managed by Google's Perception stack
// through android.hardware.perceptionhost instead of Meta's sensors HAL.
#include <dirent.h>
#include <fcntl.h>
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

#define PERCEPTION "android.hardware.perceptionhost"
#define RUNDIR "/dev/.questlhsync"

// QVR calibration paths (Qualcomm VR / Google Perception)
#define CAL_RUNTIME "/data/vendor/qvr/device_calibration.xml"
#define CAL_DEFAULT "/vendor/etc/qvr/device_calibration_default.xml"

const char *const hs_capture_name = "frida";
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

// the calibration file: runtime-refined if readable, else factory default
static int calibration_path(char *path, size_t n) {
  snprintf(path, n, "%s", CAL_RUNTIME);
  if (access(path, R_OK) == 0) return 1;
  snprintf(path, n, "%s", CAL_DEFAULT);
  return access(path, R_OK) == 0;
}

// wrap the QVR XML calibration in a JSON envelope the PC driver can identify
int hs_calibration(char **data, size_t *len, char *name, size_t n) {
  char path[512];
  if (!calibration_path(path, sizeof path)) {
    snprintf(name, n, "no QVR calibration in %s or %s", CAL_RUNTIME, CAL_DEFAULT);
    return -1;
  }
  size_t xml_len;
  char *xml = read_file(path, &xml_len);
  if (!xml || !xml_len) {
    free(xml);
    snprintf(name, n, "can't read %s", path);
    return -1;
  }
  // {"kind": "galaxy_xr", "qvr_calibration": "<xml content>"}
  // Escape the XML for JSON embedding: replace \ with \\, " with \", newlines with \n
  size_t esc_cap = xml_len * 2 + 256;
  char *esc = malloc(esc_cap);
  size_t eo = 0;
  for (size_t i = 0; i < xml_len && eo + 8 < esc_cap; i++) {
    char c = xml[i];
    if (c == '\\') { esc[eo++] = '\\'; esc[eo++] = '\\'; }
    else if (c == '"') { esc[eo++] = '\\'; esc[eo++] = '"'; }
    else if (c == '\n') { esc[eo++] = '\\'; esc[eo++] = 'n'; }
    else if (c == '\r') { /* skip */ }
    else if ((unsigned char)c < 0x20) { esc[eo++] = ' '; }
    else esc[eo++] = c;
  }
  esc[eo] = 0;

  static const char hdr[] = "{\"kind\": \"galaxy_xr\", \"qvr_calibration\": \"";
  static const char trl[] = "\"}\n";
  *len = strlen(hdr) + eo + strlen(trl);
  *data = malloc(*len + 1);
  char *o = *data;
  memcpy(o, hdr, strlen(hdr)); o += strlen(hdr);
  memcpy(o, esc, eo); o += eo;
  memcpy(o, trl, strlen(trl) + 1);

  free(esc);
  free(xml);
  const char *base = strrchr(path, '/');
  snprintf(name, n, "qvr/%s", base ? base + 1 : path);
  return 0;
}

// ---------------------------------------------------------------- camera list from QVR calibration
// The QVR XML has <Camera name="trackingA"> etc. with <CaptureDetails native_width="640" native_height="640">.
// We pick cameras named "tracking*" and "ctrl-tracking*" (the global-shutter SLAM cameras).
// Output: "id:w:h:scan,..." where scan=1 for side cameras (tracking), scan=1 for ctrl-tracking too since the Galaxy XR
// has all four tracking cameras facing outward (unlike Quest Pro where the front pair rarely saw base stations).
static int camera_list(const char *xml, size_t len, char *out, size_t cap) {
  struct { int id, w, h, scan; } c[16];
  int n = 0;
  const char *p = xml, *e = xml + len;
  out[0] = 0;

  while (n < 16 && (p = strstr(p, "<Camera ")) && p < e) {
    const char *ce = strstr(p, "</Camera>");
    if (!ce || ce > e) break;

    // extract name="..."
    const char *nm = strstr(p, "name=\"");
    if (!nm || nm > ce) { p = ce; continue; }
    nm += 6;
    const char *nme = strchr(nm, '"');
    if (!nme || nme > ce) { p = ce; continue; }
    int namelen = (int)(nme - nm);

    // only tracking and ctrl-tracking cameras
    int is_tracking = (namelen >= 8 && !strncmp(nm, "tracking", 8));
    int is_ctrl = (namelen >= 13 && !strncmp(nm, "ctrl-tracking", 13));
    if (!is_tracking && !is_ctrl) { p = ce; continue; }

    // extract native resolution from CaptureDetails
    int w = 640, h = 640;  // default for S5K931
    const char *cd = strstr(p, "native_width=\"");
    if (cd && cd < ce) w = atoi(cd + 14);
    cd = strstr(p, "native_height=\"");
    if (cd && cd < ce) h = atoi(cd + 15);

    if (w > 0 && h > 0 && w <= 4096 && h <= 4096) {
      c[n].id = n;
      c[n].w = w;
      c[n].h = h;
      c[n].scan = 1;  // scan all tracking cameras on Galaxy XR
      n++;
    }
    p = ce;
  }

  size_t o = 0;
  for (int i = 0; i < n && o + 32 < cap; i++)
    o += snprintf(out + o, cap - o, "%s%d:%d:%d:%d", i ? "," : "", c[i].id, c[i].w, c[i].h, c[i].scan);
  return n;
}

// ---------------------------------------------------------------- find perceptionhost
static pid_t hal_pid(char *name, size_t cap) {
  DIR *d = opendir("/proc");
  if (!d) return 0;
  struct dirent *e;
  pid_t found = 0, other = 0;
  snprintf(name, cap, "%s", PERCEPTION);
  while (!found && (e = readdir(d))) {
    if (e->d_name[0] < '0' || e->d_name[0] > '9') continue;
    char p[64], cmd[256];
    snprintf(p, sizeof p, "/proc/%s/cmdline", e->d_name);
    int fd = open(p, O_RDONLY);
    if (fd < 0) continue;
    ssize_t n = read(fd, cmd, sizeof cmd - 1);
    close(fd);
    if (n <= 0) continue;
    cmd[n] = 0;
    const char *base = strrchr(cmd, '/') ? strrchr(cmd, '/') + 1 : cmd;
    if (!strcmp(base, PERCEPTION)) found = atoi(e->d_name);
    else if (!other && strstr(base, "perceptionhost")) {
      other = atoi(e->d_name);
      snprintf(name, cap, "%s", base);
    }
  }
  closedir(d);
  if (found) snprintf(name, cap, "%s", PERCEPTION);
  return found ? found : other;
}

// lhsight.js with this headset's cameras filled in
static void prepare_script(char *out, size_t cap, char *note, size_t ncap) {
  char src[512], path[512], cams[512] = "";
  size_t len, cal_len;
  out[0] = 0;
  snprintf(src, sizeof src, "%s/lhsight.js", moddir);
  char *js = read_file(src, &len);
  if (!js) { snprintf(note, ncap, "can't read %s", src); return; }

  char *cal = NULL;
  if (calibration_path(path, sizeof path)) cal = read_file(path, &cal_len);
  int n = cal ? camera_list(cal, cal_len, cams, sizeof cams) : 0;
  free(cal);
  if (n) snprintf(note, ncap, "tracking cameras (id:width:height:scan) %s", cams);
  else snprintf(note, ncap, "no tracking cameras in QVR calibration: trying Galaxy XR defaults (4x 640x640)");

  char *at = strstr(js, "@CAMS@");
  snprintf(out, cap, "%s/lhsight.js", RUNDIR);
  FILE *f = fopen(out, "wb");
  if (!f) { snprintf(note, ncap, "can't write %s", out); out[0] = 0; free(js); return; }
  if (at && n) {
    fwrite(js, 1, at - js, f);
    fputs(cams, f);
    fputs(at + 6, f);
  } else if (at && !n) {
    // Galaxy XR default: 4 tracking cameras, 640x640, all scanned
    fwrite(js, 1, at - js, f);
    fputs("0:640:640:1,1:640:640:1,2:640:640:1,3:640:640:1", f);
    fputs(at + 6, f);
  } else {
    fwrite(js, 1, len, f);
  }
  fclose(f);
  chmod(out, 0644);
  free(js);
}

// stop stray frida-inject processes running lhsight.js (same as quest.c)
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
    snprintf(m, sizeof m, "stopping a stray lhsight (frida-inject pid %s)", found[i]);
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

pid_t hs_capture_start(int out_fd, char *msg, size_t n) {
  char hal_name[256];
  pid_t hal = hal_pid(hal_name, sizeof hal_name);
  if (!hal) { snprintf(msg, n, "perceptionhost not running"); return 0; }
  char frida[512], script[512], pidstr[16], note[600];
  mkdir(RUNDIR, 0755);
  mkdir(RUNDIR "/tmp", 0755);
  prepare_script(script, sizeof script, note, sizeof note);
  if (!script[0]) { snprintf(msg, n, "%s", note); return 0; }
  snprintf(frida, sizeof frida, "%s/frida-inject", moddir);
  snprintf(pidstr, sizeof pidstr, "%d", hal);
  pid_t pid = fork();
  if (pid == 0) {
    setsid();
    dup2(out_fd, 1);
    dup2(out_fd, 2);
    int nul = open("/dev/null", O_RDONLY);
    dup2(nul, 0);
    for (int fd = 3; fd < 1024; fd++) close(fd);
    setenv("TMPDIR", RUNDIR "/tmp", 1);
    chdir(RUNDIR);
    execl(frida, "frida-inject", "-p", pidstr, "-s", script, (char *)NULL);
    _exit(127);
  }
  if (pid < 0) return -1;
  snprintf(msg, n, "lhsight started in %s (pid %d); %s", hal_name, hal, note);
  return pid;
}
