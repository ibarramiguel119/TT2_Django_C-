#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>
#include <pybind11/complex.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <tuple>
#include <string>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <fcntl.h>
#include <chrono>
#include <thread>


#include <fcntl.h>      // Para abrir archivos
#include <unistd.h>     // Para leer/escribir en el puerto serie
#include <termios.h>    // Configuración del puerto serie



#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif




namespace py = pybind11;



//Definiciones de prototipos 
int abrirPuertoSerie(const std::string& puertoEspecifico);
double radianes_a_grados(double radianes);
double gradosARadianes(double grados);
std::tuple<double, double, double> sph2cart(double r, double theta, double phi);
std::vector<double> generarPuntos(int start, int end, int new_roll);

extern "C" {
    std::string floatToString(float value, int precision);
}


bool enviarDatosCaracterPorCaracter(int serial_fd, const std::string& datos);
std::string recibirDatosBloqueante(int serial_fd);
void cerrarPuertoSerie(int serial_fd);
std::vector<double> grados_a_radianes(const std::vector<int>& grados);




void printVectorOfVectors(const std::vector<std::vector<int>>& vec);
void delaySeconds(int seconds);







// Definición de la función suma
int suma(int a, int b) {
    return a + b;
}


void CalcularGrados(int Altitud, int Asimuth, int Roll ,int& GAltitude, int& GAsimut, int& GRoll ){
    double H[3];
    H[0]=Altitud;
    H[1]=Asimuth;
    H[2]=Roll;
    //Numero de grados se moveran por punto
    GAltitude=60/H[0];
    GAsimut=360/H[1];
    GRoll=210/H[2];
}

// En el siguiente arreglo almacena la distribuccion de los grados en altitude
std::vector<int>CalcularArreglo1(int GAltitude) {
    std::vector<int> array1;
    for (int i = 0; i <= 360; i += GAltitude) {
        array1.push_back(i);
    }
    return array1;
}

// En el siguiente arreglo almacena la distribuccion de los grados en Azimuth
std::vector<int>CalcularArreglo2(int GAsimut) {
    std::vector<int> array2;
    for (int i = 0; i <= 60; i += GAsimut) {
        array2.push_back(i);
    }
    return array2;
}

// En el siguiente arreglo almacena la distribuccion de los grados en Roll
std::vector<int>CalcularArreglo3(int GRoll) {
    std::vector<int> array3;
    for (int i = 0; i <= 210; i += GRoll) {
        array3.push_back(i);
    }
    return array3;
}


// En la siguiente funcion forma los puntos donde se movera el robot
std::vector<std::vector<int>> CalcularPuntosMovimiento(const std::vector<int>& array1, const std::vector<int>& array2, int radio) {
    std::vector<std::vector<int>> S_data;

    for (int i = 0; i < array2.size(); ++i) {
        for (int j = 0; j < array1.size(); ++j) {
            std::vector<int> S = {array1[j], array2[i], radio};
            S_data.push_back(S);
        }
    }
    return S_data;
}




// Función para abrir el puerto serie en Linux
int abrirPuertoSerie(const std::string& puertoEspecifico) {
    int fd = open(puertoEspecifico.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1) {
        std::cerr << "Error al abrir el puerto " << puertoEspecifico << std::endl;
        return -1;
    }
    
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);  // Velocidad de entrada
    cfsetospeed(&options, B115200);  // Velocidad de salida
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd, TCSANOW, &options);
    
    return fd;
}

// Función para cerrar el puerto serie en Linux
void cerrarPuertoSerie(int fd) {
    close(fd);
}

// Función para enviar datos a través del puerto serie en Linux
bool enviarDatosCaracterPorCaracter(int fd, const std::string& datosAEnviar) {
    for (char c : datosAEnviar) {
        if (write(fd, &c, 1) != 1) {
            return false;
        }
    }
    return true;
}

// Función para recibir datos de forma bloqueante en Linux
std::string recibirDatosBloqueante(int fd) {
    char buffer[256];
    ssize_t n = read(fd, buffer, sizeof(buffer) - 1);
    if (n > 0) {
        buffer[n] = '\0';
        return std::string(buffer);
    }
    return "";
}

//Funcion que convierte grados a radianes
std::vector<double> grados_a_radianes(const std::vector<int>& grados) {
    std::vector<double> radianes;
    radianes.reserve(grados.size()); // Reserva espacio para evitar reasignaciones

    for (int grado : grados) {
        double radian = grado * M_PI / 180.0;
        radianes.push_back(radian);
    }
    return radianes;
}


//Funcion para cordenadas Esfericas a cartecianas
std::tuple<double, double, double> sph2cart(double azimuth, double elevation, double radius) {
    double x = radius * std::cos(elevation) * std::cos(azimuth);
    double y = radius * std::cos(elevation) * std::sin(azimuth);  
    double z = radius * std::sin(elevation);
    return std::make_tuple(x, y, z);
}


//Genera puntos para la articulacion 5 
std::vector<double> generarPuntos(int inicio, int fin, int num_puntos) {

    std::vector<double> puntos;
    // Verificar el caso especial cuando num_puntos es 1
    if (num_puntos == 1) {
        // Agregar 90 grados convertido a radianes
        double radian = 90.0 * M_PI / 180.0;
        puntos.push_back(radian);
        return puntos;
    }

    // Calcular el incremento
    int incremento = (fin - inicio) / (num_puntos - 1);
    // Generar los puntos y convertirlos a radianes
    for (int i = 0; i < num_puntos; ++i) {
        double grado = inicio + i * incremento;
        double radian = grado * M_PI / 180.0; // Convertir a radianes
        puntos.push_back(radian);
    }

    return puntos;
} 

//Convierte punto flotante a String
std::string floatToString(float value, int precision) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}


//Funcion de  conversion de grados a radianes 
double radianes_a_grados(double radianes) {
    return radianes * (180.0 / M_PI);
}


double gradosARadianes(double grados) {
    return grados * M_PI / 180.0;
}



/// Imprimir los datos del vector Result para comparar en matlab  
void printVectorOfVectors(const std::vector<std::vector<int>>& vec) {
    for (const auto& innerVec : vec) {
        std::cout << '[';
        for (size_t i = 0; i < innerVec.size(); ++i) {
            std::cout << innerVec[i];
            if (i < innerVec.size() - 1) {
                std::cout << ", ";
            }
        }
        std::cout << ']' << std::endl;
    }
}

void delaySeconds(int seconds) {
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
}


// La función principal de Cinemática Inversa
std::vector<std::vector<int>> CalcularPuntosCinematicaInversa(const std::vector<std::vector<int>>& result, int si, int slider0Value, int new_roll, py::function callback) {
    std::vector<std::vector<int>> D; 
    int n = 0;
    //int l2 = 300;  // Longitud del eslabón
    int cont = 1;

    // Configuración del puerto serie en Linux
    std::string puertoEspecifico = "/dev/ttyUSB0";  // Cambiar al puerto serie específico en Linux
    int serial_fd = abrirPuertoSerie(puertoEspecifico);

    if (serial_fd == -1) {
        std::cerr << "Error al abrir el puerto" << std::endl;
        return D;  // Salir de la función si no se puede abrir el puerto
    }

    auto numerototal = si * new_roll;

    while (n <= si) {
        std::vector<int> temp = result[n];
        auto x = slider0Value;

        std::cout << "Slider value: " << slider0Value << std::endl;
        std::cout << "Temp vector: ";
        for (int elem : temp) {
            std::cout << elem << " ";
        }
        std::cout << std::endl;

        // Convertir grados a radianes
        std::vector<double> rads = grados_a_radianes(temp);
        std::cout << "Rads: " << rads[0] << " " << rads[1] << std::endl;

        // Convertir coordenadas esféricas a cartesianas
        auto resultado = sph2cart(rads[0], rads[1], temp[2]);
        double px = std::get<0>(resultado);
        double py = std::get<1>(resultado);
        double pz = std::get<2>(resultado);
        std::cout << "Cartesian coordinates (px, py, pz): " << px << ", " << py << ", " << pz << std::endl;

        // Puntos generados para q5
        std::vector<double> puntos = generarPuntos(0, 180, new_roll);

        for (double q5 : puntos) {
            auto q1 = atan(-px / py) + 3.1416;
            auto q2 = px * sin(q1) - py * cos(q1); 
            auto q3 = pz;
            auto q4 = atan(-(q2 + 515) / q3);

            std::string q1_str = floatToString(q1, 4);
            std::string q2_str = floatToString(q2, 4);
            std::string q3_str = floatToString(q3, 4);
            std::string q4_str = floatToString(q4, 4);
            std::string q5_str = floatToString(q5, 4);
            std::string datosAEnviar = q1_str + "a" + q2_str + "b" + q3_str + "c" + q4_str + "d" + q5_str + ">";

            auto grados1 = radianes_a_grados(q1);
            std::cout << "Grados de q1: " << grados1 << std::endl;

            if (enviarDatosCaracterPorCaracter(serial_fd, datosAEnviar)) {
                std::cout << "Datos enviados: " << datosAEnviar << std::endl;
                std::string datosRecibidos = recibirDatosBloqueante(serial_fd);
                if (!datosRecibidos.empty()) {
                    std::cout << "Datos recibidos: " << datosRecibidos << std::endl;
                    if (datosRecibidos == "1") {
                        py::gil_scoped_acquire acquire;
                        callback(grados1, numerototal);
                        py::gil_scoped_release release;
                    }
                } else {
                    std::cout << "No se recibieron datos." << std::endl;
                }
            } else {
                std::cerr << "Error al enviar datos." << std::endl;
            }
        }
        D.push_back(temp);
        n++;
        cont++;
        if (cont == x + 2) {
            cont = 1;
        }
    }
    cerrarPuertoSerie(serial_fd);
    return D;
}





void procesarDatos(int slider1Value, int slider0Value, int slider2Value,int new_roll,py::function callback) {
    int GAltitude, GAsimut, GRoll;
    //int radioEsfera = 299;

    CalcularGrados(slider1Value, slider0Value, slider2Value, GAltitude, GAsimut, GRoll);
    std::cout << "Grados en Altitude: " << GAltitude << std::endl;
    std::cout << "Grados en Asimuth: " << GAsimut << std::endl;
    std::cout << "Grados en Roll:" << GRoll << std::endl;

    //Muestra el arreglo de distribuccion de angulos de Azimuth
    std::vector<int> resultado1 = CalcularArreglo1(GAsimut);
    // Imprimir el contenido del vector devuelto
    std::cout << "Los datos del arreglo 1:" << std::endl;
    for (int value1 : resultado1) {
        std::cout << value1 << " ";
    }
    std::cout << std::endl;

    //Muestra el arreglo de distribuccion de angulos de Altitude
    std::vector<int> resultado2 = CalcularArreglo2(GAltitude);
    // Imprimir el contenido del vector devuelto
    std::cout << "Los datos del arreglo 2:" << std::endl;
    for (int value2 : resultado2) {
        std::cout << value2 << " ";
    }
    std::cout << std::endl;

    //Muestra el arreglo de distribuccion de grados de Roll
    std::vector<int> resultado3 = CalcularArreglo3(GRoll);
    // Imprimir el contenido del vector devuelto
    std::cout << "Los datos del arreglo 3:" << std::endl;
    for (int value3 : resultado3) {
        std::cout << value3 << " ";
    }
    std::cout << std::endl;

    // Esta funcion calcula los puntos de posicionamiento de la circunferencia
    std::vector<std::vector<int>> result = CalcularPuntosMovimiento(resultado1, resultado2, slider2Value);
    printVectorOfVectors(result);

    // Imprimir el resultado
    // Calculo del tamaño del tamaño de la forma en se movera el robot 
    int si = (resultado1.size() - 1) * (resultado2.size() - 1);
    std::cout << si << std::endl;

    ///Imprimir los datos de vector D por separado
    //CalcularPuntosCinematicaInversa(result, si,callback); 
    CalcularPuntosCinematicaInversa(result, si,slider0Value,new_roll,callback); 
}








// Módulo que expone las funciones del Robot 
PYBIND11_MODULE(algoritmos_robot, m) {
    m.def("suma", &suma, "Una función que suma dos números");

    m.def("calcular_grados", [](int Altitud, int Asimuth, int Roll) {
        int GAltitude, GAsimut, GRoll;
        CalcularGrados(Altitud, Asimuth, Roll, GAltitude, GAsimut, GRoll);
        // Retorna los resultados como una tupla de Python
        return py::make_tuple(GAltitude, GAsimut, GRoll);
    }, "Calculates degrees for Altitude, Asimuth, and Roll");

    m.def("CalcularArreglo1", &CalcularArreglo1, "Función para calcular arreglo 1");

    m.def("CalcularArreglo2", &CalcularArreglo2, "Función para calcular arreglo 2");

    m.def("CalcularArreglo3", &CalcularArreglo3, "Función para calcular arreglo 3");

    m.def("procesarDatos", &procesarDatos,"Esta funcion simplifica la llamadas en el programa principal");
}