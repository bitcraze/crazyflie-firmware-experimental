#include "wand_interface.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "log.h"

#define DEBUG_MODULE "WAND"
#include "debug.h"

#define WAND_P2P_PORT 0x01

#define GRASP_DIST 0.3f
#define GRASP_THRESHOLD 30.0f
#define BUILD_RATE 3.0f
#define LOSS_TIMEOUT_MS 1500

typedef struct {
    uint8_t id;
    float x, y, z;
    float dx, dy, dz;
} WandLinePacket;

static float attemptScore = 0.0f;
static bool grasped = false;
static uint32_t lastPacketTime = 0;
static float graspRange = 0.0f;
static WandLinePacket lastPkt;

static logVarId_t idX;
static logVarId_t idY;
static logVarId_t idZ;

static float vector_norm(float x, float y, float z) {
    return sqrtf(x * x + y * y + z * z);
}

static void cross_product(float ax, float ay, float az,
                          float bx, float by, float bz,
                          float *rx, float *ry, float *rz)
{
    *rx = ay * bz - az * by;
    *ry = az * bx - ax * bz;
    *rz = ax * by - ay * bx;
}

void wandInit(void)
{
    idX = logGetVarId("stateEstimate", "x");
    idY = logGetVarId("stateEstimate", "y");
    idZ = logGetVarId("stateEstimate", "z");

    attemptScore = 0.0f;
    grasped = false;
    lastPacketTime = 0;
    graspRange = 0.0f;
    memset(&lastPkt, 0, sizeof(lastPkt));
}

void wandHandleP2PPacket(P2PPacket *p)
{
    if (p->port != WAND_P2P_PORT) {
        return;
    }

    if (p->size != 1 + 6 * sizeof(float)) {
        return;
    }

    lastPacketTime = xTaskGetTickCount();

    WandLinePacket pkt;
    pkt.id = p->data[0];
    memcpy(&pkt.x, &p->data[1], sizeof(float));
    memcpy(&pkt.y, &p->data[1 + 4], sizeof(float));
    memcpy(&pkt.z, &p->data[1 + 8], sizeof(float));
    memcpy(&pkt.dx, &p->data[1 + 12], sizeof(float));
    memcpy(&pkt.dy, &p->data[1 + 16], sizeof(float));
    memcpy(&pkt.dz, &p->data[1 + 20], sizeof(float));

    lastPkt = pkt;

    float rx = logGetFloat(idX);
    float ry = logGetFloat(idY);
    float rz = logGetFloat(idZ);

    float vx = rx - pkt.x;
    float vy = ry - pkt.y;
    float vz = rz - pkt.z;

    float cx, cy, cz;
    cross_product(vx, vy, vz, pkt.dx, pkt.dy, pkt.dz, &cx, &cy, &cz);

    float dist = vector_norm(cx, cy, cz);

    if (!grasped) {
        if (dist < GRASP_DIST) {
            float prevScore = attemptScore;
            attemptScore = fminf(attemptScore + BUILD_RATE, 100.0f);
            if (attemptScore > prevScore) {
                // Shine orange when grasp score increases and not grasped
                extern void ledSetRGB(uint8_t r, uint8_t g, uint8_t b);
                ledSetRGB(0x60, 0x30, 0x00); // ORANGE_LED
            }
        }

        if (attemptScore > GRASP_THRESHOLD) {
            grasped = true;
            graspRange = vx * pkt.dx + vy * pkt.dy + vz * pkt.dz;
            if (graspRange < 0) {
                graspRange = 0.2f;
            }
            DEBUG_PRINT("GRASPED! Wand=%d range=%.5f\n", pkt.id, (double)graspRange);
        }
    }
}

void wandUpdate(uint32_t nowTicks)
{
    if (grasped && (nowTicks - lastPacketTime > M2T(LOSS_TIMEOUT_MS))) {
        grasped = false;
        attemptScore = 0.0f;
        DEBUG_PRINT("RELEASED (lost wand signal)\n");
    }
}

bool wandIsGrasped(void)
{
    return grasped;
}

void wandGetSetpoint(float *x, float *y, float *z)
{
    if (x != NULL) {
        *x = lastPkt.x + graspRange * lastPkt.dx;
    }
    if (y != NULL) {
        *y = lastPkt.y + graspRange * lastPkt.dy;
    }
    if (z != NULL) {
        *z = lastPkt.z + graspRange * lastPkt.dz;
    }
}
