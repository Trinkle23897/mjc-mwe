#include <bits/stdc++.h>
#include <mjxmacro.h>
#include <mujoco.h>

int main(int argc, char const *argv[])
{
    char error[1000];
    mjModel* m = mj_loadXML(argv[1], 0, error, 1000);
    assert(m);
    return 0;
}