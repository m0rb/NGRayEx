/* raycast.c - fixed-point DDA raycaster mapping wall slices onto the Neo
 * Geo's hardware sprite shrinker */
#include "raycast.h"
#include "config.h"
#include "map.h"

/* ---- 16.16 fixed point ---------------------------------------------- */
typedef s32 fix;
#define FBITS 16
#define FONE  (1 << FBITS)
#define FIX(x) ((fix)((x) * (double)FONE))   /* constant initializers only  */

static inline fix fmul(fix a, fix b) { return (fix)(((int64_t)a * b) >> FBITS); }
static inline fix fdiv(fix a, fix b) { return (fix)(((int64_t)a << FBITS) / b); }
static inline fix fabsx(fix a)       { return a < 0 ? -a : a; }
static inline fix fmul_fast(fix a, fix b) { return (a >> 8) * (b >> 8); }
static inline fix fmul_frac(fix frac, fix d) { return (fix)(((u32)frac * (u32)(d >> 8)) >> 8); }
 
static inline fix recip(fix b) {
    u32 ab = (b < 0) ? (u32)(-b) : (u32)b;
    if (ab < (u32)(FONE >> 8)) ab = (FONE >> 8);   /* clamp |b| >= 1/256 */
    return (fix)(0xFFFFFFFFu / ab);
}

#define FBIG (1 << 28)            
#define FMIN (FONE >> 6)          /* clamp tiny distances                   */
 
static fix posX, posY;           /* world position (1.0 == one map cell)    */
static fix dirX, dirY;           /* facing direction (unit)                 */
static fix planeX, planeY;       /* camera plane (sets FOV; |plane|~0.66)   */
static fix dirTabX[NANG], dirTabY[NANG];   /* facing vectors, precomputed   */
static fix plnTabX[NANG], plnTabY[NANG];   /* camera planes, precomputed    */
static int pang;                           /* facing angle index            */
static fix camTab[NUM_COLS];               /* cameraX per column            */

static u16 scb2buf[NUM_COLS];    /* (HSHRINK<<8)|vshrink                    */
static u16 scb3buf[NUM_COLS];    /* Y/size word                             */
static u8  palbuf[NUM_COLS];     /* desired palette this frame              */
static u8  curpal[NUM_COLS];     /* palette currently in VRAM (cache)       */

static void set_facing(void) {
    dirX = dirTabX[pang]; dirY = dirTabY[pang];
    planeX = plnTabX[pang]; planeY = plnTabY[pang];
}

void rc_init(void) {
    posX = FIX(8.5); posY = FIX(12.5);   /* open floor, clear of all walls     */
    {                                    /* build facing tables once           */
        fix cs = FIX(ANG_COS), sn = FIX(ANG_SIN), fov = FIX(FOV);
        fix dx = FIX(0.0), dy = FIX(-1.0);
        for (int a = 0; a < NANG; a++) {
            dirTabX[a] = dx;             dirTabY[a] = dy;
            plnTabX[a] = fmul(-dy, fov); plnTabY[a] = fmul(dx, fov);
            fix ndx = fmul(dx, cs) - fmul(dy, sn);
            fix ndy = fmul(dx, sn) + fmul(dy, cs);
            dx = ndx; dy = ndy;
        }
    }
    pang = 0;                            /* face north across the hall         */
    set_facing();
    for (int x = 0; x < NUM_COLS; x++)
        camTab[x] = (fix)((2 * FONE * x) / (NUM_COLS - 1)) - FONE;
    for (int c = 0; c < NUM_COLS; c++) curpal[c] = 0xFF; /* force first write */
}

static void try_move(fix dx, fix dy) {
    fix nx = posX + dx, ny = posY + dy;
    if (!map_at(nx >> FBITS, posY >> FBITS)) posX = nx;
    if (!map_at(posX >> FBITS, ny >> FBITS)) posY = ny;
}

void rc_input(u8 pressed) {
    enum { UP=1, DOWN=2, LEFT=4, RIGHT=8, A=16 };
    fix spd = FIX(MOVE_SPEED);
    if (pressed & UP)   try_move(fmul(dirX, spd), fmul(dirY, spd));
    if (pressed & DOWN) try_move(-fmul(dirX, spd), -fmul(dirY, spd));
    if (pressed & A) {                              /* strafe with A held    */
        if (pressed & LEFT)  try_move(-fmul(planeX, spd), -fmul(planeY, spd));
        if (pressed & RIGHT) try_move( fmul(planeX, spd),  fmul(planeY, spd));
    } else {
        if (pressed & LEFT)  { pang = (pang - ROT_STEP) & (NANG - 1); set_facing(); }
        if (pressed & RIGHT) { pang = (pang + ROT_STEP) & (NANG - 1); set_facing(); }
    }
}

void rc_player_cell(int *cx, int *cy) {
    *cx = posX >> FBITS;
    *cy = posY >> FBITS;
}

void rc_render(void) {
    for (int x = 0; x < NUM_COLS; x++) {
        /* camera x in [-1, +1] across the screen */
        fix cameraX = camTab[x];
        fix rayX = dirX + fmul_fast(planeX, cameraX);
        fix rayY = dirY + fmul_fast(planeY, cameraX);

        int mapX = posX >> FBITS;
        int mapY = posY >> FBITS;

        fix ddX = recip(rayX);
        fix ddY = recip(rayY);

        int stepX, stepY;
        fix sideX, sideY;
        if (rayX < 0) { stepX = -1; sideX = fmul_frac(posX - (mapX << FBITS), ddX); }
        else          { stepX =  1; sideX = fmul_frac(((mapX + 1) << FBITS) - posX, ddX); }
        if (rayY < 0) { stepY = -1; sideY = fmul_frac(posY - (mapY << FBITS), ddY); }
        else          { stepY =  1; sideY = fmul_frac(((mapY + 1) << FBITS) - posY, ddY); }

        int side = 0;                       /* 0 = hit on X grid line (N/S)  */
        for (;;) {
            if (sideX < sideY) { sideX += ddX; mapX += stepX; side = 0; }
            else               { sideY += ddY; mapY += stepY; side = 1; }
            if (map_at(mapX, mapY)) break;
        }

        fix perp = (side == 0) ? (sideX - ddX) : (sideY - ddY);
        if (perp < FMIN) perp = FMIN;

        int h =  (int)((u32)(WALLH << FBITS) / (u32)perp);  /* slice height px */
        if (h < 1)     h = 1;
        if (h > MAX_H) h = MAX_H;

        int top = (SCRH - h) / 2;           /* >=0 because h<=SCRH           */
        int vsh = h - 1;                    /* on-screen px = vshrink+1      */
        if (vsh < 0)   vsh = 0;
        if (vsh > 255) vsh = 255;

        scb2buf[x] = (u16)((HSHRINK << 8) | (vsh & 0xFF));
		
        scb3buf[x] = scb3_word(top, 0, WALL_WIN);

        /* distance shading */
        int band = ((MAX_H - h) * DEPTH_BANDS) / MAX_H;
        if (band < 0) band = 0;
        if (band >= DEPTH_BANDS) band = DEPTH_BANDS - 1;
        palbuf[x] = (u8)(PAL_DEPTH_BASE + (side ? DEPTH_BANDS : 0) + band);
    }
}

#define PAL_COLS_PER_FRAME 10
static int pal_cursor = 0;

void rc_blit(void) {
    /* stream vertical shrink for every wall slice */
    vram_addr(VRAM_SCB2 + WALL_BASE);
    vram_mod(1);
    for (int c = 0; c < NUM_COLS; c++) vram_w_nop(scb2buf[c]);

    /* stream Y/size */
    vram_addr(VRAM_SCB3 + WALL_BASE);
    vram_mod(1);
    for (int c = 0; c < NUM_COLS; c++) vram_w_nop(scb3buf[c]);

    /* directional shading, throttled */
    int painted = 0;
    for (int n = 0; n < NUM_COLS; n++) {
        int c = pal_cursor + n;
        if (c >= NUM_COLS) c -= NUM_COLS;
        if (palbuf[c] == curpal[c]) continue;
        u16 spr = WALL_BASE + c;
        vram_addr(VRAM_SCB1 + spr * 64 + 1);
        vram_mod(2);
        u16 attr = (u16)(palbuf[c] << 8);
        for (int t = 0; t < WALL_WIN; t++) vram_w_nop(attr);
        curpal[c] = palbuf[c];
        if (++painted >= PAL_COLS_PER_FRAME) {
            pal_cursor = c + 1;
            if (pal_cursor >= NUM_COLS) pal_cursor = 0;
            return;
        }
    }
    pal_cursor = 0;
}
