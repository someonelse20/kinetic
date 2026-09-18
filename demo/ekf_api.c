/*
 * Kinetic EKF API Bridge - Standalone HTTP server
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>

typedef struct { float data[3][3]; } matrix_t;
typedef struct { float data[4]; } matrix4_t;

typedef struct {
    matrix4_t ekf_state;
    matrix_t g_ref;
    matrix_t m_ref;
} imu_t;

static imu_t g_imu = {0};

static void init_matrix(matrix_t *m) {
    for (int i=0; i<3; i++) for (int j=0; j<3; j++) m->data[i][j] = 0.0f;
}

static void fill_matrix(matrix_t *m, float val) {
    for (int i=0; i<3; i++) for (int j=0; j<3; j++) m->data[i][j] = val;
}

static matrix_t *quat_to_rot_matrix(matrix4_t *q) {
    matrix_t *R = malloc(sizeof(matrix_t));
    if (!R) return NULL;
    float x = q->data[0], y = q->data[1], z = q->data[2], w = q->data[3];
    R->data[0][0] = 1 - 2*y*y - 2*z*z; R->data[0][1] = 2*x*y - 2*z*w; R->data[0][2] = 2*x*z + 2*y*w;
    R->data[1][0] = 2*x*y + 2*z*w; R->data[1][1] = 1 - 2*x*x - 2*z*z; R->data[1][2] = 2*y*z - 2*x*w;
    R->data[2][0] = 2*x*z - 2*y*w; R->data[2][1] = 2*y*z + 2*x*w; R->data[2][2] = 1 - 2*x*x - 2*y*y;
    return R;
}

static matrix_t *trans_matrix_alloc(matrix_t *m) {
    if (!m) return NULL;
    matrix_t *t = malloc(sizeof(matrix_t));
    if (!t) return NULL;
    for (int i=0; i<3; i++) for (int j=0; j<3; j++) t->data[j][i] = m->data[i][j];
    return t;
}

static void free_matrix(matrix_t *m) { if (m) free(m); }

static matrix_t *mul_matrix_alloc(matrix_t *A, matrix_t *B) {
    if (!A || !B) return NULL;
    matrix_t *C = malloc(sizeof(matrix_t));
    if (!C) return NULL;
    for (int i=0; i<3; i++) for (int j=0; j<3; j++) {
        C->data[i][j] = 0; for (int k=0; k<3; k++) C->data[i][j] += A->data[i][k] * B->data[k][j];
    }
    return C;
}

static matrix_t *scale_matrix_alloc(matrix_t *A, float s) {
    if (!A) return NULL;
    matrix_t *B = malloc(sizeof(matrix_t));
    if (!B) return NULL;
    for (int i=0; i<3; i++) for (int j=0; j<3; j++) B->data[i][j] = A->data[i][j] * s;
    return B;
}

static void add_matrix(matrix_t *A, matrix_t *B, matrix_t *C) {
    if (!A || !B || !C) return;
    for (int i=0; i<3; i++) for (int j=0; j<3; j++) C->data[i][j] = A->data[i][j] + B->data[i][j];
}

static void sub_matrix(matrix_t *A, matrix_t *B, matrix_t *C) {
    if (!A || !B || !C) return;
    for (int i=0; i<3; i++) for (int j=0; j<3; j++) C->data[i][j] = A->data[i][j] - B->data[i][j];
}

static matrix_t *sub_matrix_alloc(matrix_t *A, matrix_t *B) {
    if (!A || !B) return NULL;
    matrix_t *C = malloc(sizeof(matrix_t));
    if (!C) return NULL;
    sub_matrix(A, B, C); return C;
}

static matrix_t *add_matrix_alloc(matrix_t *A, matrix_t *B) {
    if (!A || !B) return NULL;
    matrix_t *C = malloc(sizeof(matrix_t));
    if (!C) return NULL;
    add_matrix(A, B, C); return C;
}

static matrix_t *move_matrix(matrix_t *A, matrix_t *B) {
    if (!A || !B) return NULL;
    for (int i=0; i<3; i++) for (int j=0; j<3; j++) B->data[i][j] = A->data[i][j];
    return B;
}

static matrix_t *copy_matrix(matrix_t *A) {
    if (!A) return NULL;
    matrix_t *B = malloc(sizeof(matrix_t));
    if (!B) return NULL;
    move_matrix(A, B); return B;
}

static void imu_init(imu_t *imu) { memset(imu, 0, sizeof(imu_t)); }

static void imu_update(imu_t *imu, const float *gyro, const float *accel) {
    matrix4_t q = {0}; memcpy(&q, &imu->ekf_state, sizeof(matrix4_t));
    float dt = 0.1f;
    float wx = gyro[0], wy = gyro[1], wz = gyro[2];
    float qx = q.data[0], qy = q.data[1], qz = q.data[2], qw = q.data[3];
    float dq[4] = {wx*(qy*qw - qz*qx), wy*(qx*qw + qy*qz), wz*(qx*qy - qz*qw), 0};
    q.data[0] += dq[0]*dt; q.data[1] += dq[1]*dt; q.data[2] += dq[2]*dt; q.data[3] += dq[3]*dt;
    float norm = sqrtf(q.data[0]*q.data[0] + q.data[1]*q.data[1] + q.data[2]*q.data[2] + q.data[3]*q.data[3]);
    q.data[0] /= norm; q.data[1] /= norm; q.data[2] /= norm; q.data[3] /= norm;
    
    matrix_t *R = quat_to_rot_matrix(&q);
    if (!R) return;
    
    matrix_t *e_acc = mul_matrix_alloc(R, &imu->g_ref);
    matrix_t *e_mag = mul_matrix_alloc(R, &imu->m_ref);
    matrix_t *innovation = sub_matrix_alloc(e_acc, (matrix_t *)accel);
    matrix_t *K = scale_matrix_alloc(trans_matrix_alloc(R), 0.1f);
    matrix_t *dq_result = mul_matrix_alloc(K, innovation);
    
    float new_q[4];
    new_q[0] = q.data[0] + dq_result->data[0][0]; new_q[1] = q.data[1] + dq_result->data[0][1];
    new_q[2] = q.data[2] + dq_result->data[0][2]; new_q[3] = q.data[3];
    float new_norm = sqrtf(new_q[0]*new_q[0] + new_q[1]*new_q[1] + new_q[2]*new_q[2] + new_q[3]*new_q[3]);
    new_q[0] /= new_norm; new_q[1] /= new_norm; new_q[2] /= new_norm; new_q[3] /= new_norm;
    memcpy(&q, new_q, sizeof(matrix4_t));
    memcpy(&imu->ekf_state, &q, sizeof(matrix4_t));
    
    free_matrix(R); free_matrix(e_acc); free_matrix(e_mag); free_matrix(innovation); free_matrix(K); free_matrix(dq_result);
}

typedef struct { char *method; char *path; char *body; size_t body_len; } http_req_t;

static void send_json(int fd, const char *status, const char *body) {
    char header[512];
    size_t body_len = strlen(body);
    snprintf(header, sizeof(header), "HTTP/1.1 %s\r\nContent-Type: application/json\r\nContent-Length: %zu\r\n\r\n", status, body_len);
    write(fd, header, strlen(header));
    write(fd, body, body_len);
    fflush(stdout);
}

static void send_err(int fd, int code, const char *msg) {
    char body[256]; snprintf(body, sizeof(body), "{\"error\":\"%s\"}", msg);
    send_json(fd, "Error", body);
}

static void imu_update_from_json(imu_t *imu, const char *json) {
    if (!json || !*json) return;
    
    const char *start = strstr(json, "\"gyro_x\":");
    if (!start) return;
    
    char gx[32], gy[32], gz[32], ax[32], ay[32], az[32];
    const char *comma = strchr(start, ',');
    if (!comma) comma = strchr(start + 9, '}');
    strncpy(gx, start + 9, comma - start - 9); gx[comma - start - 9] = '\0';
    
    start = strchr(comma, '"') + 1;
    comma = strchr(start, ',');
    if (!comma) comma = strchr(start + 9, '}');
    strncpy(gy, start + 9, comma - start - 9); gy[comma - start - 9] = '\0';
    
    start = strchr(comma, '"') + 1;
    comma = strchr(start, ',');
    if (!comma) comma = strchr(start + 9, '}');
    strncpy(gz, start + 9, comma - start - 9); gz[comma - start - 9] = '\0';
    
    start = strchr(comma, '"') + 1;
    comma = strchr(start, ',');
    if (!comma) comma = strchr(start + 9, '}');
    strncpy(ax, start + 9, comma - start - 9); ax[comma - start - 9] = '\0';
    
    start = strchr(comma, '"') + 1;
    comma = strchr(start, ',');
    if (!comma) comma = strchr(start + 9, '}');
    strncpy(ay, start + 9, comma - start - 9); ay[comma - start - 9] = '\0';
    
    start = strchr(comma, '"') + 1;
    comma = strchr(start, ',');
    if (!comma) comma = strchr(start + 9, '}');
    strncpy(az, start + 9, comma - start - 9); az[comma - start - 9] = '\0';
    
    float g[3] = {atof(gx), atof(gy), atof(gz)};
    float a[3] = {atof(ax), atof(ay), atof(az)};
    imu_update(imu, g, a);
}

static void imu_get_state_json(imu_t *imu, int fd) {
    matrix4_t q = {0}; memcpy(&q, &imu->ekf_state, sizeof(matrix4_t));
    matrix_t *R = quat_to_rot_matrix(&q);
    if (!R) { send_err(fd, 500, "{\"error\":\"Failed\"}"); return; }
    float qx = q.data[0], qy = q.data[1], qz = q.data[2], qw = q.data[3];
    float r00 = R->data[0][0], r01 = R->data[0][1], r02 = R->data[0][2];
    float r10 = R->data[1][0], r11 = R->data[1][1], r12 = R->data[1][2];
    float r20 = R->data[2][0], r21 = R->data[2][1], r22 = R->data[2][2];
    float roll = atan2f(2*(qx*qw + qy*qz), 1.0f - 2*(qx*qx + qy*qy));
    float pitch = atan2f(2*(qy*qw - qz*qx), 1.0f - 2*(qx*qx + qz*qz));
    float yaw = atan2f(2*(qz*qw + qx*qy), 1.0f - 2*(qy*qy + qz*qz));
    char json[4096];
    snprintf(json, sizeof(json), "{\"quaternion\":[%.6f,%.6f,%.6f,%.6f],\"roll\":%.6f,\"pitch\":%.6f,\"yaw\":%.6f,\"r00\":%.6f,\"r01\":%.6f,\"r02\":%.6f,\"r10\":%.6f,\"r11\":%.6f,\"r12\":%.6f,\"r20\":%.6f,\"r21\":%.6f,\"r22\":%.6f}", qx, qy, qz, qw, roll, pitch, yaw, r00, r01, r02, r10, r11, r12, r20, r21, r22);
    free_matrix(R);
    send_json(fd, "OK", json);
}

static void handle_request(int fd, http_req_t *req) {
    if (strcmp(req->method, "GET") == 0) {
        if (strcmp(req->path, "/state") == 0) { imu_get_state_json(&g_imu, fd); }
        else if (strcmp(req->path, "/") == 0 || strcmp(req->path, "/index.html") == 0) {
            FILE *f = fopen("index.html", "r");
            if (f) { char buf[4096]; size_t n, tot=0; while ((n = fread(buf, 1, sizeof(buf)-1, f)) > 0) { buf[n-1] = '\0'; tot += n; if (tot >= sizeof(buf)-1) break; } fclose(f); send_json(fd, "OK", buf); }
            else send_err(fd, 404, "Not found");
        }
        else if (strcmp(req->path, "/ekf_update") == 0) { imu_update_from_json(&g_imu, req->body); imu_get_state_json(&g_imu, fd); }
        else send_err(fd, 404, "Not found");
    }
    else if (strcmp(req->method, "POST") == 0 && strcmp(req->path, "/ekf_update") == 0) {
        printf("Processing POST /ekf_update\n"); fflush(stdout);
        if (!req->body) { send_err(fd, 400, "No body"); }
        else { imu_update_from_json(&g_imu, req->body); }
        imu_get_state_json(&g_imu, fd);
    }
    else send_err(fd, 405, "Method not allowed");
    free(req->method); free(req->path); free(req->body);
}

static int parse_request(char *line, http_req_t *req) {
    req->method = req->path = req->body = NULL; req->body_len = 0;
    char *start = line;
    const char *method = strsep(&start, " "); if (!method) return 0; req->method = strdup(method);
    const char *path = strsep(&start, " "); if (!path) return 0; req->path = strdup(path);
    return 1;
}

int main(void) {
    int srv, cli; struct sockaddr_in svr_addr, cli_addr; socklen_t clilen; char buf[8192]; char *line;
    signal(SIGINT, SIG_IGN); signal(SIGTERM, SIG_IGN);
    srv = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1; setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    memset(&svr_addr, 0, sizeof(svr_addr)); svr_addr.sin_family = AF_INET; svr_addr.sin_addr.s_addr = INADDR_ANY; svr_addr.sin_port = htons(8085);
    if (bind(srv, (struct sockaddr*)&svr_addr, sizeof(svr_addr)) < 0) { perror("bind"); return 1; }
    if (listen(srv, 5) < 0) { perror("listen"); return 1; }
    printf("Kinetic EKF API running on http://0.0.0.0:8085");
    printf("  GET /state - Get EKF state");
    printf("  POST /ekf_update - Update EKF");
    printf("  GET / - Serve HTML");
    while (1) {
        clilen = sizeof(cli_addr); cli = accept(srv, (struct sockaddr*)&cli_addr, &clilen);
        if (cli < 0) { perror("accept"); continue; }
        memset(buf, 0, sizeof(buf));
        if (read(cli, buf, sizeof(buf)-1) <= 0) { close(cli); continue; }
        line = buf; http_req_t req; memset(&req, 0, sizeof(req));
        if (!parse_request(line, &req)) { send_err(cli, 400, "Invalid request"); close(cli); continue; }
        if (req.method && strcmp(req.method, "POST") == 0 && req.body_len == 0) {
            while (*line && *line != '\n' && *line != '\r') line++;
            while (*line == '\r' || *line == '\n') line++;
            char *end; size_t clen = (size_t)strtol(line, &end, 10);
            if (*line != '\n' && *line != '\r' && *line != '\0') { send_err(cli, 400, "Invalid Content-Length"); close(cli); continue; }
            if (clen == 0) break;
            req.body = malloc(clen + 1);
            if (!req.body) { send_err(cli, 500, "Memory error"); close(cli); continue; }
            if (read(cli, req.body, clen) != (ssize_t)clen) { send_err(cli, 400, "Incomplete request"); close(cli); continue; }
            req.body[clen] = '\0';
        }
        handle_request(cli, &req); close(cli);
    }
    close(srv); return 0;
}