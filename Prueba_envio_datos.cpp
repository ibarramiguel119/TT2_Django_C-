#include <iostream>
#include <fcntl.h>      // Para `open`
#include <unistd.h>     // Para `write`, `close`, `usleep`
#include <termios.h>    // Para `termios`
#include <cstring>      // Para `strlen`

// Función para abrir y configurar el puerto serial
int configurarPuertoSerie(const char* puerto) {
    // Abrir el puerto serial
    int fd = open(puerto, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) {
        std::cerr << "Error al abrir el puerto serial." << std::endl;
        return -1;
    }

    // Configurar el puerto serial
    struct termios options;
    tcgetattr(fd, &options);

    // Configuraciones del puerto
    options.c_cflag &= ~PARENB;        // Sin paridad
    options.c_cflag &= ~CSTOPB;        // 1 bit de parada
    options.c_cflag &= ~CSIZE;         // Limpiar tamaño de bits
    options.c_cflag |= CS8;            // 8 bits de datos
    options.c_cflag |= CLOCAL;         // Ignorar líneas de control de módem
    options.c_cflag |= CREAD;          // Habilitar la recepción de datos

    // Configurar velocidad de transmisión a 115200
    cfsetispeed(&options, B230400);
    cfsetospeed(&options, B230400);

    // Aplicar configuraciones inmediatamente
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

// Función para enviar una cadena de texto por el puerto serial
void enviarCadena(int fd, const char* cadena) {
    int longitud = strlen(cadena);
    for (int i = 0; i < longitud; i++) {
        // Enviar carácter por carácter
        if (write(fd, &cadena[i], 1) != 1) {
            std::cerr << "Error al enviar el carácter: " << cadena[i] << std::endl;
            break;
        }
        usleep(100000);  // Espera de 1 ms entre caracteres (ajustable)
    }

    // Esperar a que se vacíe el buffer de transmisión
    tcdrain(fd);
}

int main() {
    const char* puerto = "/dev/ttyUSB0";  // Ajusta el puerto según tu dispositivo
    const char* mensaje = "1.5708a150.0000b0.0000c-1.5708d1.5708>";

    // Configurar el puerto serial
    int fd = configurarPuertoSerie(puerto);
    if (fd == -1) {
        return 1;
    }

    // Enviar el mensaje en un ciclo infinito
    while (true) {
        enviarCadena(fd, mensaje);
        std::cout << "Mensaje enviado: " << mensaje << std::endl;

        usleep(1000000);  // Espera de 1 segundo entre mensajes (ajustable)
    }

    // Cerrar el puerto serial (nunca se alcanza en este ciclo infinito)
    close(fd);
    return 0;
}
