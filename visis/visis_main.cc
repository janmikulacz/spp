/**
 * File:   visis_main.cc
 *
 * Date:   11.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/visis_program.h"

int main(
    int argc,
    const char *const *argv
) {
    visis::ProgramOptions po;
    char c = visis::ParseProgramOptions(argc, argv, po);
    if (c == 'h') {
        return EXIT_SUCCESS;
    } else if (c == 'e') {
        return EXIT_FAILURE;
    } else {
        return visis::VisisProgram(po);
    }
}
