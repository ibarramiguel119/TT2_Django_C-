#include <pybind11/pybind11.h>

namespace py = pybind11;

// Definición de la función suma
int suma(int a, int b) {
    return a + b;
}

// Módulo que expone la función suma
PYBIND11_MODULE(algoritmos_robot, m) {
    m.def("suma", &suma, "Una función que suma dos números");
}