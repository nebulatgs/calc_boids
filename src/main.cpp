// TI-84 CE Boid Simulation
//
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include "Vector.h"
#include <tice.h>
#include "v2d.h"
#include <graphx.h>
#include <gfx/gfx.h>
#include <keypadc.h>
#include <fileioc.h>

const int factor = 4096;
const int width = 320 * factor, height = 240 * factor;

int vis = 70 * factor;
int sqVis = vis*vis;

int maxSpeed = 8 * factor;
//float maxForce = 0.2;
float maxForce = 0.4f;

int boidCount = 12;

int align = 1 * factor;
int seperate = 1 * factor;
int cohere = 1 * factor;
int cursorForce = 1 * factor;
bool bounce = 0;
gfx_rletsprite_t *particle_rlet, *mouse_rlet;
v2d mouseVec;
v2d mouseVel;
uint16_t hues [16] = {224,225,226,227,228,228,230,231,199,167,135,103,71,79,87,95};


struct Boid {
// Construct new boid with random location and position with index of _index
    Boid(uint16_t _index) {
        index = _index;
        pos = v2d(rand() % width, rand() % height);
        vel.randomize(maxSpeed);
    }
// Override the == operator to compare indices
    bool operator == (const Boid& compare)
    {
        return(this->index == compare.index);
    }

    uint8_t index;
    v2d pos;
    v2d vel;
    v2d acc;
    v2d aln;
    v2d csn;
    v2d sep;
    Vector <uint16_t> neighbors;
    Vector <int> dists;

// Calculate forces based on neighbors
    void flock(Vector <Boid*>& boids) {
    // Zero all force vectors and neighbors
        uint16_t total = 0;
        acc.zero();
        aln.zero();
        csn.zero();
        sep.zero();
        neighbors.Clear();
        dists.Clear();

    // Loop over all boids and calculate distances
        for (uint16_t i = 0; i < boidCount; i++) {
            Boid* target = boids[i];
        // Skip ourself
            if (target == this) continue;
        // Distance to boid
            int d;
        // Skip recalculation of distance if possible
            if (index > target->index) {
                int j = target->neighbors.Find(index);
                if (j + 1) {
                    d = target->dists[j];
                }
                else continue;
            }
        // Calculate the distance to the boid
            else d = pos.sqrDist(target->pos);
        // Push found neighbors
            if (d <= sqVis) {
                neighbors.PushBack(index);
                dists.PushBack(d);
                aln += target->vel;
                csn += target->pos;
                sep = (sep + (pos - target->pos)) / d;
                total++;
            }
        }

        if (total > 0) {
        // Limit alignment force
            aln.setLen(maxSpeed);
            aln -= vel;
            aln.limit(maxForce);
        // Limit cohesion force
            csn = (csn / total) - pos;
            csn.setLen(maxSpeed);
            csn -= vel;
            csn.limit(maxForce);
        // Limit seperation force
            sep.setLen(maxSpeed);
            sep -= vel;
            sep.limit(maxForce);
        }
    // Apply all forces to acceleration
        acc = (acc + aln) * align;
        acc = (acc + csn) * cohere;
        acc = (acc + sep) * seperate;

    }

    void update() {
        vel += acc;
        vel.limit(maxSpeed);
        pos += vel;
        if (pos.x < 0) pos.x = width;
        if (pos.x > width) pos.x = 0;
        if (pos.y < 0) pos.y = height;
        if (pos.y > height) pos.y = 0;

        if (mouseVec.x < 0) mouseVec.x = width;
        if (mouseVec.x > width) mouseVec.x = 0;
        if (mouseVec.y < 0) mouseVec.y = height;
        if (mouseVec.y > height) mouseVec.y = 0;
    }

    void draw(){
        v2d lineEnd((pos + vel * 5));
        gfx_Line(pos.x, pos.y, lineEnd.x, lineEnd.y);
        gfx_RLETSprite(particle_rlet, pos.x/factor - particle_width/2, pos.y/factor - particle_width/2);
        gfx_RLETSprite(mouse_rlet, mouseVec.x/factor - mouse_width/2, mouseVec.y/factor - mouse_width/2);
    }

    void cursor(bool explode){
        int d = mouseVec.sqrDist(pos);
        mouseVel = v2d(mouseVec);
        mouseVel -= pos;
        mouseVel.setLen(10000/d || 1);
        mouseVel.limit(cursorForce);
        if(explode) acc += mouseVel; else acc -= mouseVel;
    }

};

void drawAll(Vector<Boid*>& flock){
    for (uint16_t i = 1; i < (boidCount-1); i++) {
    // Update boid position
        flock[i]->update();
    // Draw boid
        flock[i]->draw();
    }
}

int main(int argc, char *argv[])
{
    real_t * os_boidCount;
    real_t * os_vision;
    ti_RclVar(TI_REAL_TYPE, ti_O, (void**)&os_boidCount);
    ti_RclVar(TI_REAL_TYPE, ti_V, (void**)&os_vision);
    boidCount = (uint16_t)os_RealToInt24(os_boidCount) > 0 ? (uint16_t)os_RealToInt24(os_boidCount) : 10;
    sqVis = (int)os_RealToInt24(os_vision) > 0 ? (int)os_RealToInt24(os_vision) : 30;
    sqVis *= factor;
    sqVis *= sqVis;
    //boidCount = argc > 0 ? *argv[0] : 12;
// Initialize particle sprite
    gfx_SetTransparentColor(0);
    particle_rlet = gfx_ConvertMallocRLETSprite(particle);
    mouse_rlet = gfx_ConvertMallocRLETSprite(mouse);
    //mouseVec.randomize();
    mouseVec = v2d(width/2, height/2);
// Seed RNG with current time
    srand((unsigned int)rtc_Time());
    Vector<Boid*> flock;

// Initialize graphics drawing
    gfx_Begin();
    gfx_SetColor(75);
    gfx_SetDrawBuffer();

// Populate vector with new boids
    for (uint16_t i = 0; i < boidCount; i++) {
        flock.PushBack(new Boid(i));
    }

// Simulate until a key is pressed
    float frame = 0;
    gfx_palette[74] = gfx_RGBTo1555(31, 30, 27);
    gfx_palette[75] = gfx_RGBTo1555(67, 66, 64);
    kb_key_t arrowKey;
    kb_key_t triggerKey;
    kb_key_t turboKey;
    while (kb_Data[6] != kb_Clear) {
    // Calculate forces on all boids
        for (uint16_t i = 1; i < boidCount-1 ; i++) {
            arrowKey = kb_Data[7];
            triggerKey = kb_Data[2];
            turboKey = kb_Data[3];
            if (turboKey != kb_0) flock[i]->flock(flock);
            if (triggerKey == kb_Alpha) flock[i]->cursor(1); else if (triggerKey == kb_Math) flock[i]->cursor(0);
            switch(arrowKey) {
                case kb_Up:
                    mouseVec.y-=factor;
                    break;
                case kb_Down:
                    mouseVec.y+=factor;
                    break;
                case kb_Left:
                    mouseVec.x-=factor;
                    break;
                case kb_Right:
                    mouseVec.x+=factor;
                    break;
            }
            kb_Scan();
        }
    // Zero the graphics buffer
        gfx_FillScreen(74);
    // Update boid positions and draw
        gfx_palette[127] = gfx_palette[hues[(uint16_t)((sinf(frame/10.0f) + 1) * 7.5f)]];
        drawAll(flock);
        gfx_SwapDraw();
        frame++;
    }

    free(particle_rlet);
    free(mouse_rlet);
    gfx_End();
    return 0;
}