/*
 * Kinetic EKF API Bridge
 * Exposes EKF state and update function to JavaScript via HTTP
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <ctype.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "kin_types.h"
#include "kin_math.h"
#include "kin_ekf.h"
#include "kin_imu.h"

/* Global EKF instance */
static imu_t g_imu = {0};

/* HTTP Request structure */
typedef struct {
    char *method;
    char *path;
    char *body;
    size_t body_len;
} http_req_t;

static imu_t *get_imu(void) {
    if (!imu_init(&g_imu, NULL, NULL)) {
        return &g_imu;
    }
    return NULL;
}

static void send_json_response(int client_fd, const char *status, const char *body) {
    char status_line[128];
    snprintf(status_line, sizeof(status_line), "HTTP/1.1 %s\r\n", status);
    
    const char *headers = 
        "Content-Type: application/json\r\n"
        "Content-Length: %zu\r\n"
        "Connection: close\r\n"
        "\r\n";
    
    size_t body_len = strlen(body);
    size_t header_len = strlen(headers);
    
    char buffer[8192];
    snprintf(buffer, sizeof(buffer), "%s%zu\r\n", status_line, body_len);
    
    write(client_fd, buffer, strlen(buffer));
    write(client_fd, headers, header_len);
    write(client_fd, body, body_len);
}

static void send_error_response(int client_fd, int code, const char *msg) {
    char body[256];
    snprintf(body, sizeof(body), "{\"error\":\"%s\"}", msg);
    send_json_response(client_fd, "Error", body);
}

static void imu_update_from_json(imu_t *imu, const char *json) {
    /* Parse JSON: {"gyro_x":0.1,"gyro_y":-0.05,"gyro_z":0.02,"acc_x":9.81,"acc_y":0.1,"acc_z":9.8} */
    const char *start = strstr(json, "\"gyro_x\":");
    if (!start) return;
    
    char gyro_x[32], gyro_y[32], gyro_z[32];
    char acc_x[32], acc_y[32], acc_z[32];
    
    /* Extract gyro_x */
    const char *comma = strchr(start, ',');
    if (!comma) comma = strchr(start + 9, '}');
    strncpy(gyro_x, start + 9, comma - start - 9);
    gyro_x[comma - start - 9] = '\0';
    
    /* Extract gyro_y */
    start = strchr(comma, '\"') + 1;
    comma = strchr(start, ',');
    if (!comma) comma = strchr(start + 9, '}');
    strncpy(gyro_y, start + 9, comma - start - 9);
    gyro_y[comma - start - 9] = '\0';
    
    /* Extract gyro_z */
    start = strchr(comma, '\"') + 1;
    comma = strchr(start, ',');
    if (!comma) comma = strchr(start + 9, '}');
    strncpy(gyro_z, start + 9, comma - start - 9);
    gyro_z[comma - start - 9] = '\0';
    
    /* Extract acc_x */
    start = strchr(comma, '\"') + 1;
    comma = strchr(start, ',');
    if (!comma) comma = strchr(start + 9, '}');
    strncpy(acc_x, start + 9, comma - start - 9);
    acc_x[comma - start - 9] = '\0';
    
    /* Extract acc_y */
    start = strchr(comma, '\"') + 1;
    comma = strchr(start, ',');
    if (!comma) comma = strchr(start + 9, '}');
    strncpy(acc_y, start + 9, comma - start - 9);
    acc_y[comma - start - 9] = '\0';
    
    /* Extract acc_z */
    start = strchr(comma, '\"') + 1;
    comma = strchr(start, ',');
    if (!comma) comma = strchr(start + 9, '}');
    strncpy(acc_z, start + 9, comma - start - 9);
    acc_z[comma - start - 9] = '\0';
    
    float gx = atof(gyro_x), gy = atof(gyro_y), gz = atof(gyro_z);
    float ax = atof(acc_x), ay = atof(acc_y), az = atof(acc_z);
    
    /* imu_update(imu, gyro, accel, mag) */
    imu_update(imu, &gx, &ax, &az);
}

static void imu_get_state_as_json(imu_t *imu, char *buffer, size_t buf_size) {
    matrix_t *q = imu->ekf.state;
    matrix_t *rot = quat_to_rot_matrix(q);
    
    float qx = q->data[X], qy = q->data[Y], qz = q->data[Z], qw = q->data[W];
    
    float r00 = rot->data[0], r01 = rot->data[1], r02 = rot->data[2];
    float r10 = rot->data[3], r11 = rot->data[4], r12 = rot->data[5];
    float r20 = rot->data[6], r21 = rot->data[7], r22 = rot->data[8];
    
    float roll = atan2(2*(qx*qw + qy*qz), 1 - 2*(qx*qx + qy*qy));
    float pitch = atan2(2*(qy*qw - qz*qx), 1 - 2*(qx*qx + qz*qz));
    float yaw = atan2(2*(qz*qw + qx*qy), 1 - 2*(qy*qy + qz*qz));
    
    snprintf(buffer, buf_size,
        "{\"quaternion\":[%.6f,%.6f,%.6f,%.6f],"
        "{\"roll\":%.6f,\"pitch\":%.6f,\"yaw\":%.6f},"
        "{\"r00\":%.6f,\"r01\":%.6f,\"r02\":%.6f,"
         "r10\":%.6f,\"r11\":%.6f,\"r12\":%.6f,"
         "r20\":%.6f,\"r21\":%.6f,\"r22\":%.6f}"
    , qx, qy, qz, qw, roll, pitch, yaw,
      r00, r01, r02, r10, r11, r12, r20, r21, r22);
    
    free_matrix(rot);
}

static void imu_get_state_json(imu_t *imu, int client_fd) {
    char json[4096];
    imu_get_state_as_json(imu, json, sizeof(json));
    send_json_response(client_fd, "OK", json);
}

static void handle_request(int client_fd, http_req_t *req) {
    if (strcmp(req->method, "GET") == 0) {
        if (strcmp(req->path, "/state") == 0) {
            imu_t *imu = get_imu();
            if (imu) {
                imu_get_state_json(imu, client_fd);
            } else {
                send_error_response(client_fd, 500, "Failed to initialize EKF");
            }
        } else if (strcmp(req->path, "/") == 0 || strcmp(req->path, "/index.html") == 0) {
            FILE *f = fopen("index.html", "r");
            if (f) {
                char buffer[4096];
                size_t bytes_read, total = 0;
                while ((bytes_read = fread(buffer, 1, sizeof(buffer) - 1, f)) > 0) {
                    buffer[bytes_read - 1] = '\0';
                    total += bytes_read;
                    if (total >= sizeof(buffer) - 1) break;
                }
                fclose(f);
                send_json_response(client_fd, "OK", buffer);
            } else {
                send_error_response(client_fd, 404, "Not found");
            }
        } else if (strcmp(req->path, "/ekf_update") == 0) {
            imu_t *imu = get_imu();
            if (imu) {
                imu_update_from_json(imu, req->body);
                imu_get_state_json(imu, client_fd);
            } else {
                send_error_response(client_fd, 500, "Failed to initialize EKF");
            }
        } else {
            send_error_response(client_fd, 404, "Not found");
        }
    } else if (strcmp(req->method, "POST") == 0) {
        if (strcmp(req->path, "/ekf_update") == 0) {
            imu_t *imu = get_imu();
            if (imu) {
                imu_update_from_json(imu, req->body);
                imu_get_state_json(imu, client_fd);
            } else {
                send_error_response(client_fd, 500, "Failed to initialize EKF");
            }
        } else {
            send_error_response(client_fd, 404, "Not found");
        }
    } else {
        send_error_response(client_fd, 405, "Method not allowed");
    }
    
    free(req->method);
    free(req->path);
    free(req->body);
}

static int parse_request(char *line, http_req_t *req) {
    req->method = NULL;
    req->path = NULL;
    req->body = NULL;
    req->body_len = 0;
    
    char *start = line;
    const char *method = strsep(&start, " ");
    if (!method) return 0;
    req->method = strdup(method);
    
    const char *path = strsep(&start, " ");
    if (!path) return 0;
    req->path = strdup(path);
    
    return 1;
}

int main(int argc, char *argv[]) {
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len;
    char buffer[8192];
    char *line;
    
    signal(SIGINT, SIG_IGN);
    signal(SIGTERM, SIG_IGN);
    
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket");
        return 1;
    }
    
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt");
        close(server_fd);
        return 1;
    }
    
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(8082);
    
    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind");
        close(server_fd);
        return 1;
    }
    
    if (listen(server_fd, 5) < 0) {
        perror("listen");
        close(server_fd);
        return 1;
    }
    
    printf("Kinetic EKF API server running on http://0.0.0.0:8082\n");
    printf("  GET /state - Get current EKF state\n");
    printf("  POST /ekf_update - Update EKF with sensor data\n");
    printf("  GET / - Serve index.html\n");
    
    while (1) {
        client_len = sizeof(client_addr);
        client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
        if (client_fd < 0) {
            perror("accept");
            continue;
        }
        
        memset(buffer, 0, sizeof(buffer));
        if (read(client_fd, buffer, sizeof(buffer) - 1) <= 0) {
            close(client_fd);
            continue;
        }
        
        line = buffer;
        http_req_t req;
        memset(&req, 0, sizeof(req));
        
        if (!parse_request(line, &req)) {
            send_error_response(client_fd, 400, "Invalid request");
            close(client_fd);
            continue;
        }
        
        if (req.method && strcmp(req.method, "POST") == 0 && req.body_len == 0) {
            while (*line && *line != '\n' && *line != '\r') line++;
            while (*line == '\r' || *line == '\n') line++;
            
            char *end;
            size_t content_len = (size_t)strtol(line, &end, 10);
            
            if (*line != '\n') {
                send_error_response(client_fd, 400, "Invalid Content-Length");
                close(client_fd);
                continue;
            }
            
            if (content_len == 0) break;
            
            req.body = (char *)malloc(content_len + 1);
            if (!req.body) {
                send_error_response(client_fd, 500, "Memory allocation failed");
                close(client_fd);
                continue;
            }
            
            if (read(client_fd, req.body, content_len) != (ssize_t)content_len) {
                send_error_response(client_fd, 400, "Incomplete request");
                close(client_fd);
                continue;
            }
            req.body[content_len] = '\0';
        }
        
        handle_request(client_fd, &req);
        close(client_fd);
    }
    
    close(server_fd);
    imu_deinit(&g_imu);
    return 0;
}
