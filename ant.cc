#include <bits/stdc++.h>
#include <mjxmacro.h>
#include <mujoco.h>

class AntEnv {
    mjModel *m;
    mjData *d;
    mjtNum *init_qpos, *init_qvel;
    double reset_noise_scale_, min_contact_force, max_contact_force;
    char error[1000];
    std::mt19937 gen_;
    std::uniform_real_distribution<> dist_qpos_;
    std::normal_distribution<> dist_qvel_;
    int obs_size;
    mjtNum *obs_buf;
public:
    AntEnv(const char *xml):
            m(mj_loadXML(xml, 0, error, 1000)), d(mj_makeData(m)),
            init_qpos(new mjtNum[m->nq]), init_qvel(new mjtNum[m->nv]),
            reset_noise_scale_(0.1), min_contact_force(-1.0),
            max_contact_force(1.0), gen_(0),
            dist_qpos_(-reset_noise_scale_, reset_noise_scale_),
            dist_qvel_(0, 1),
            obs_size(m->nq - 2 + m->nv + 6 * m->nbody),
            obs_buf(new mjtNum[obs_size])
    {
        assert(m);
        assert(d);
        memcpy(init_qpos, d->qpos, sizeof(mjtNum) * m->nq);
        memcpy(init_qvel, d->qvel, sizeof(mjtNum) * m->nv);
        puts("init");
        printf("init_qpos: %d | ", m->nq);
        for (int i = 0; i < m->nq; ++i) {
            printf("%.3f ", init_qpos[i]);
        }
        puts("");
        printf("init_qvel: %d | ", m->nv);
        for (int i = 0; i < m->nv; ++i) {
            printf("%.3f ", init_qvel[i]);
        }
        printf("\nobservation_space %d", obs_size);
        printf("\naction_space %d | ", m->nu);
        for (int i = 0; i < 2 * m->nu; ++i) {
            printf("%.2f ", m->actuator_ctrlrange[i]);
        }
        puts("\ninit done");
    }

    ~AntEnv() {
        mj_deleteData(d);
        mj_deleteModel(m);
    }

    void reset() {
        mj_resetData(m, d);
        reset_model();
        mj_forward(m, d);
        get_obs();
    }

    // special for ant

    void reset_model() {
        // noise_low = -self._reset_noise_scale
        // noise_high = self._reset_noise_scale

        // qpos = self.init_qpos + self.np_random.uniform(
        //     low=noise_low, high=noise_high, size=self.model.nq
        // )
        // qvel = (
        //     self.init_qvel
        //     + self._reset_noise_scale * self.np_random.standard_normal(self.model.nv)
        // )
        // self.set_state(qpos, qvel)
        for (int i = 0; i < m->nq; ++i) {
            d->qpos[i] = init_qpos[i] + dist_qpos_(gen_);
        }
        for (int i = 0; i < m->nv; ++i) {
            d->qvel[i] = init_qvel[i] + reset_noise_scale_ * dist_qvel_(gen_);
        }
    }

    void get_obs() {
        // @property
        // def contact_forces(self):
        //     raw_contact_forces = self.data.cfrc_ext
        //     min_value, max_value = self._contact_force_range
        //     contact_forces = np.clip(raw_contact_forces, min_value, max_value)
        //     return contact_forces

        // position = self.data.qpos.flat.copy()
        // velocity = self.data.qvel.flat.copy()
        // contact_force = self.contact_forces.flat.copy()
        // if self._exclude_current_positions_from_observation:
        //     position = position[2:]

        // observations = np.concatenate((position, velocity, contact_force))
        int ptr = 0;
        for (int i = 2; i < m->nq; ++i) {
            obs_buf[ptr++] = d->qpos[i];
        }
        for (int i = 0; i < m->nv; ++i) {
            obs_buf[ptr++] = d->qvel[i];
        }
        for (int i = 0; i < 6 * m->nbody; ++i) {
            mjtNum x = d->cfrc_ext[i];
            x = std::min(max_contact_force, x);
            x = std::max(min_contact_force, x);
            obs_buf[ptr++] = x;
        }
        assert(ptr == obs_size);
        printf("obs: ");
        for (int i = 0; i < obs_size; ++i) {
            if (i % 6 == 0) puts("");
            printf("%.8f\t", obs_buf[i]);
        }
    }
};


int main(int argc, char const *argv[])
{
    AntEnv e(argv[1]);
    e.reset();
    return 0;
}
