#include "Viewer.h"

int main(int argc, char** argv)
{
    Viewer window("Viewer", 800, 600);

    window.load_data("../data/pointsets/0469_BRISO_8_D1_PPCCP_Rep1_D1_08_49_ARCH2021-09-06__2021-09-17__4231.csv");

    return window.run();
}
