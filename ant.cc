#include <bits/stdc++.h>
#include <mjxmacro.h>
#include <mujoco.h>

int main(int argc, char const *argv[])
{
    char error[1000];
    mjModel* m = mj_loadXML(argv[1], 0, error, 1000);
    assert(m);
    mjData *d = mj_makeData(m);
    assert(d);
    mjtNum *init_qpos = new mjtNum[m->nq];
    memcpy(init_qpos, d->qpos, sizeof(mjtNum) * m->nq);
    mjtNum *init_qvel = new mjtNum[m->nv];
    memcpy(init_qvel, d->qvel, sizeof(mjtNum) * m->nv);
    printf("nq: %d | ", m->nq);
    for (int i = 0; i < m->nq; ++i) {
        printf("%.3f ", init_qpos[i]);
    }
    puts("");
    printf("nv: %d | ", m->nv);
    for (int i = 0; i < m->nv; ++i) {
        printf("%.3f ", init_qvel[i]);
    }
    puts("");
    return 0;
}