// SPDX-License-Identifier: BSD-2-Clause-Patent
#ifndef PAGANI_BUTTONS_H
#define PAGANI_BUTTONS_H
#include <stdint.h>
#include <PaganiPeripherals.h>
// Key bits. Report 2 (Reports.h) carries Power, Volume Up and Volume Down in these bits. The Plus key is not reported as a
// key: it flips the iPhone ring/silent switch, the Telephony Ring Select element in bit 4 (1 = ring, 0 = silent).
// Bit 0, Consumer Menu (Home), has no key on this board; only the input script (InputScript.h) presses it.
enum {
    PAGANI_KEY_MENU=1, PAGANI_KEY_POWER=2, PAGANI_KEY_VOLUME_UP=4, PAGANI_KEY_VOLUME_DOWN=8, PAGANI_KEY_PLUS=16,
    PAGANI_KEYS_PON=PAGANI_KEY_POWER|PAGANI_KEY_VOLUME_UP,
    PAGANI_KEYS_PMIC=PAGANI_KEYS_PON|PAGANI_KEY_VOLUME_DOWN,
    PAGANI_REPORT_RING=16,
    PAGANI_KEYS_POLL_MS=8,
    // PON RT_STS is debounced by the PMIC; Linux pm8941-pwrkey reports it directly and, for pmk8350-class PON, ignores
    // presses within twice the PON debounce after a release.
    PAGANI_PON_STABLE_MS=16, PAGANI_PON_RELOCK_MS=32,
    // Raw GPIO inputs (Volume Down on the PMIC, the Plus key on TLMM): the Android gpio-keys debounce-interval.
    PAGANI_GPIO_STABLE_MS=32,
};
// Raw pressed mask: PON bits 7/6 are Power/Volume Up; the PMIC GPIO and the Plus key's TLMM input are active low.
static inline uint8_t pagani_buttons(uint8_t pon,uint8_t gpio,uint32_t plus) {
#ifdef PAGANI_PON_RESIN_VOLUME_DOWN
    // Boards that wire RESIN to Volume Down and Volume Up to the PMIC GPIO (Xiaomi piano).
    return (uint8_t)(((pon&PAGANI_PON_POWER)?PAGANI_KEY_POWER:0)|((pon&PAGANI_PON_VOLUME_UP)?PAGANI_KEY_VOLUME_DOWN:0)|
                     ((gpio&PAGANI_GPIO_VOLUME_DOWN)?0:PAGANI_KEY_VOLUME_UP)|((plus&1)?0:PAGANI_KEY_PLUS));
#endif
    return (uint8_t)(((pon&PAGANI_PON_POWER)?PAGANI_KEY_POWER:0)|((pon&PAGANI_PON_VOLUME_UP)?PAGANI_KEY_VOLUME_UP:0)|
                     ((gpio&PAGANI_GPIO_VOLUME_DOWN)?0:PAGANI_KEY_VOLUME_DOWN)|((plus&1)?0:PAGANI_KEY_PLUS));
}
struct pagani_button_filter {
    uint8_t candidate,stable,seen,known;
    uint64_t since[5],until[5];   // by key bit number 1..4: start of the current raw level, earliest accepted press
};
static inline unsigned pagani_key_stable_ms(unsigned bit) {
    return ((1u<<bit)&PAGANI_KEYS_PON)?PAGANI_PON_STABLE_MS:PAGANI_GPIO_STABLE_MS;
}
// Per-key stable state, both edges: a key changes after its raw level stayed the same for its stable time, and a PON key is
// not pressed again within PAGANI_PON_RELOCK_MS of its release. Keys outside `valid` (a failed read) are released at once and
// restart their debounce. `known` marks keys whose current state has been confirmed since then.
static inline uint8_t pagani_buttons_filter(struct pagani_button_filter *f,uint8_t valid,uint8_t mask,uint64_t ms) {
    for (unsigned bit=1;bit<=4;++bit) {
        uint8_t key=(uint8_t)(1u<<bit),raw=(uint8_t)(mask&key);
        if (!(valid&key)) {
            f->stable&=(uint8_t)~key;f->seen&=(uint8_t)~key;f->known&=(uint8_t)~key;
            continue;
        }
        if (!(f->seen&key) || (f->candidate&key)!=raw || ms<f->since[bit]) {
            f->seen|=key;f->candidate=(uint8_t)((f->candidate&~key)|raw);f->since[bit]=ms;
            continue;
        }
        if (ms-f->since[bit]<pagani_key_stable_ms(bit)) continue;
        if ((f->stable&key)!=raw) {
            if (raw) {
                if (ms<f->until[bit]) continue;
                f->stable|=key;
            } else {
                f->stable&=(uint8_t)~key;
                f->until[bit]=(key&PAGANI_KEYS_PON)?ms+PAGANI_PON_RELOCK_MS:0;
            }
        }
        f->known|=key;
    }
    return f->stable;
}
// The ring/silent switch driven by the Plus key: each confirmed press flips it, like moving the iPhone switch. It starts at
// ring and is armed only by a confirmed release, so a key held (or a line stuck low) since boot never silences the phone.
struct pagani_ringer { uint8_t ring,down,armed; };
static inline uint8_t pagani_ringer_update(struct pagani_ringer *r,const struct pagani_button_filter *f) {
    uint8_t down=(f->stable&PAGANI_KEY_PLUS)!=0;
    if (!(f->known&PAGANI_KEY_PLUS)) r->armed=0;
    else if (!down) r->armed=1;
    else if (!r->down && r->armed) r->ring^=1;
    r->down=down;
    return r->ring;
}
struct pagani_keys { struct pagani_button_filter filter; struct pagani_ringer ringer; };
static inline struct pagani_keys pagani_keys_initial(void) {
    struct pagani_keys keys={{0,0,0,0,{0,0,0,0,0},{0,0,0,0,0}},{1,0,0}};
    return keys;
}
// One poll: the report-2 byte with the debounced Power/Volume keys and the ring/silent state.
static inline uint8_t pagani_keys_step(struct pagani_keys *k,uint8_t valid,uint8_t mask,uint64_t ms) {
    uint8_t stable=pagani_buttons_filter(&k->filter,valid,mask,ms);
    uint8_t ring=pagani_ringer_update(&k->ringer,&k->filter);
    return (uint8_t)((stable&PAGANI_KEYS_PMIC)|(ring?PAGANI_REPORT_RING:0));
}
// TLMM CFG for the Plus key (Android pinctrl): gpio function, no pull (external pull-up), input, 6 mA; other bits kept.
static inline uint32_t pagani_plus_key_config(uint32_t saved) {
    return (saved&~0x3ffu)|((uint32_t)PAGANI_PLUS_KEY_DRIVE<<6);
}
// KEYS diagnostic sample: PMIC GPIO in bits 0..7, PON in 8..15, Plus key input level (1 released) in 16, ring state in 17,
// Plus key configured in 18.
static inline uint32_t pagani_keys_sample(uint8_t pon,uint8_t gpio,uint32_t plus,uint8_t ring,int present) {
    return ((uint32_t)pon<<8)|gpio|((plus&1)<<16)|((uint32_t)(ring!=0)<<17)|((uint32_t)(present!=0)<<18);
}
#endif
