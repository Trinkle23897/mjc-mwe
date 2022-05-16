#include <bits/stdc++.h>
#include <mjxmacro.h>
#include <mujoco.h>


class Env {
    mjModel *m;
    mjData *d;
    char error[1000];
public:
    Env(const char *xml): m(mj_loadXML(xml, 0, error, 1000)), d(mj_makeData(m))
    {
        assert(m);
        assert(d);
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_UNKNOWN, i);
            if (s == nullptr) break;
            printf("mjOBJ_UNKNOWN %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_BODY, i);
            if (s == nullptr) break;
            printf("mjOBJ_BODY %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_XBODY, i);
            if (s == nullptr) break;
            printf("mjOBJ_XBODY %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_JOINT, i);
            if (s == nullptr) break;
            printf("mjOBJ_JOINT %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_DOF, i);
            if (s == nullptr) break;
            printf("mjOBJ_DOF %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_GEOM, i);
            if (s == nullptr) break;
            printf("mjOBJ_GEOM %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_SITE, i);
            if (s == nullptr) break;
            printf("mjOBJ_SITE %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_CAMERA, i);
            if (s == nullptr) break;
            printf("mjOBJ_CAMERA %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_LIGHT, i);
            if (s == nullptr) break;
            printf("mjOBJ_LIGHT %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_MESH, i);
            if (s == nullptr) break;
            printf("mjOBJ_MESH %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_SKIN, i);
            if (s == nullptr) break;
            printf("mjOBJ_SKIN %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_HFIELD, i);
            if (s == nullptr) break;
            printf("mjOBJ_HFIELD %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_TEXTURE, i);
            if (s == nullptr) break;
            printf("mjOBJ_TEXTURE %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_MATERIAL, i);
            if (s == nullptr) break;
            printf("mjOBJ_MATERIAL %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_PAIR, i);
            if (s == nullptr) break;
            printf("mjOBJ_PAIR %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_EXCLUDE, i);
            if (s == nullptr) break;
            printf("mjOBJ_EXCLUDE %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_EQUALITY, i);
            if (s == nullptr) break;
            printf("mjOBJ_EQUALITY %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_TENDON, i);
            if (s == nullptr) break;
            printf("mjOBJ_TENDON %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_ACTUATOR, i);
            if (s == nullptr) break;
            printf("mjOBJ_ACTUATOR %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_SENSOR, i);
            if (s == nullptr) break;
            printf("mjOBJ_SENSOR %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_NUMERIC, i);
            if (s == nullptr) break;
            printf("mjOBJ_NUMERIC %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_TEXT, i);
            if (s == nullptr) break;
            printf("mjOBJ_TEXT %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_TUPLE, i);
            if (s == nullptr) break;
            printf("mjOBJ_TUPLE %d %s\n", i, s);
        }
        for (int i = 0;; ++i) {
            const auto *s = mj_id2name(m, mjOBJ_KEY, i);
            if (s == nullptr) break;
            printf("mjOBJ_KEY %d %s\n", i, s);
        }
    }

    ~Env() {
        mj_deleteData(d);
        mj_deleteModel(m);
    }
};

int main(int argc, char const *argv[])
{
    Env e(argv[1]);
    return 0;
}
