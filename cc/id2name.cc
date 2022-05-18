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

#define L(attr) \
        for (int i = 0;; ++i) { \
            const auto *s = mj_id2name(m, attr, i); \
            if (s == nullptr) break; \
            printf(#attr" %d %s\n", i, s); \
        }

#define P(attr) \
        printf(#attr": "); \
        for (int i = 0; i < m->njnt; ++i) { \
            printf("%d ", m->attr[i]); \
        } \
        puts("");

        L(mjOBJ_UNKNOWN)
        L(mjOBJ_BODY)
        L(mjOBJ_XBODY)
        L(mjOBJ_JOINT)
        L(mjOBJ_DOF)
        L(mjOBJ_GEOM)
        L(mjOBJ_SITE)
        L(mjOBJ_CAMERA)
        L(mjOBJ_LIGHT)
        L(mjOBJ_MESH)
        L(mjOBJ_SKIN)
        L(mjOBJ_HFIELD)
        L(mjOBJ_TEXTURE)
        L(mjOBJ_MATERIAL)
        L(mjOBJ_PAIR)
        L(mjOBJ_EXCLUDE)
        L(mjOBJ_EQUALITY)
        L(mjOBJ_TENDON)
        L(mjOBJ_ACTUATOR)
        L(mjOBJ_SENSOR)
        L(mjOBJ_NUMERIC)
        L(mjOBJ_TEXT)
        L(mjOBJ_TUPLE)
        L(mjOBJ_KEY)

        P(jnt_type)
        P(jnt_qposadr)
        P(jnt_dofadr)
        P(jnt_bodyid)
        P(jnt_group)
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
