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

const uint16_t width = 320, height = 240;
uint16_t vis = 70;
uint16_t sqVis = vis*vis;

uint16_t maxSpeed = 8;
//float maxForce = 0.2;
float maxForce = 0.4f;

uint16_t boidCount = 12;

uint16_t align = 1;
uint16_t seperate = 1;
uint16_t cohere = 1;
uint16_t cursorForce = 1;
bool bounce = 0;
gfx_rletsprite_t* particle_rlet;

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
    Vector <float> dists;

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
            float d;
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
    }

    void draw(){
        v2d lineEnd((pos + vel * 5));
        gfx_Line(pos.x, pos.y, lineEnd.x, lineEnd.y);
        gfx_RLETSprite(particle_rlet, pos.x - particle_width/2, pos.y - particle_width/2);
    }

    void cursor(bool explode){
        v2d cursorVec = v2d(width/2, height/2);
        float d = cursorVec.sqrDist(pos);
        cursorVec -= pos;
        cursorVec.setLen(10000/d || 1);
        cursorVec.limit(cursorForce);
        if(explode) acc += cursorVec; else acc -= cursorVec;
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

int main(void)
{
// Initialize particle sprite
    gfx_SetTransparentColor(0);
    particle_rlet = gfx_ConvertMallocRLETSprite(particle);
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
    kb_key_t key;
    while (kb_Data[1] != kb_2nd) {
    // Calculate forces on all boids
        for (uint16_t i = 1; i < boidCount-1 ; i++) {
            key = kb_Data[7];
            flock[i]->flock(flock);
            if (key == kb_Up) flock[i]->cursor(1); else if (key == kb_Down) flock[i]->cursor(0);
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
    gfx_End();
    return 0;
}