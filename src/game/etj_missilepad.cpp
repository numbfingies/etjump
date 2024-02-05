/*
 * MIT License
 *
 * Copyright (c) 2023 ETJump team <zero@etjump.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "g_local.h"
#include "etj_missilepad.h"

namespace ETJump {
void Missilepad::touch(gentity_t *self, gentity_t *other) {
  // could check for classnames here but MOD is likely the simplest
  // and marginally faster than string compare
  // we CANNOT check for ent->s.weapon here
  // because a client might be holding the weapon
  if (!(other->methodOfDeath == MOD_GPG40 || other->methodOfDeath == MOD_M7 ||
        other->methodOfDeath == MOD_GRENADE_LAUNCHER ||
        other->methodOfDeath == MOD_GRENADE_PINEAPPLE)) {
    return;
  }

  G_UseTargets(self, other);

  vec3_t noiseOrigin;

  if (!VectorCompare(self->r.currentOrigin, vec3_origin)) {
    VectorCopy(self->r.currentOrigin, noiseOrigin);
  } else {
    const vec3_t bmodelCenter = {(self->r.absmax[0] + self->r.absmin[0]) / 2,
                                 (self->r.absmax[1] + self->r.absmin[1]) / 2,
                                 (self->r.absmax[2] + self->r.absmin[2]) / 2};

    VectorCopy(bmodelCenter, noiseOrigin);
  }

  if (self->noise_index) {
    const auto te =
        soundEvent(noiseOrigin, EV_GENERAL_SOUND_VOLUME, self->noise_index);
    te->s.onFireStart = self->s.onFireStart;
  }
}

void Missilepad::use(gentity_t *ent) {
  if (ent->r.linked) {
    trap_UnlinkEntity(ent);
  } else {
    trap_LinkEntity(ent);
  }
}

void Missilepad::spawn(gentity_t *ent) {
  trap_SetBrushModel(ent, ent->model);
  InitMover(ent);
  VectorCopy(ent->s.origin, ent->s.pos.trBase);
  VectorCopy(ent->s.origin, ent->r.currentOrigin);

  ent->use = [](gentity_t *ent, gentity_t *other, gentity_t *activator) {
    Missilepad::use(ent);
  };

  if (ent->spawnflags & static_cast<int>(Spawnflags::StartInvis)) {
    trap_UnlinkEntity(ent);
  }

  char *s;
  if (G_SpawnString("noise", "NOSOUND", &s)) {
    char buffer[MAX_QPATH];
    Q_strncpyz(buffer, s, sizeof(buffer));
    ent->noise_index = G_SoundIndex(buffer);
  }

  G_SpawnInt("volume", "255", &ent->s.onFireStart);
  if (!ent->s.onFireStart) {
    ent->s.onFireStart = 255;
  }

  G_SpawnFloat("scale", "1.0", &ent->speed);

  ent->touch = [](gentity_t *ent, gentity_t *activator, trace_t *trace) {
    Missilepad::touch(ent, activator);
  };
}
} // namespace ETJump
