#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <tuple>
#include <sys/stat.h>

std::string get_color_name(char color) {
    switch (color) {
    case 'r':
        return "red";
    case 'g':
        return "green";
    case 'b':
        return "blue";
    default:
        return "";
    }
}

std::tuple<int, int, std::string> decode_message(char* message){
    std::stringstream stream;
    stream << message;
    double x,y;
    char color;
    stream >> x >> y >> color;

    return {x, y, get_color_name(color)};
}



int main() {
    int fd;
    const char *fifo = "pipe";
    constexpr std::size_t message_size{6};
    mkfifo(fifo, 0666);
    fd = open(fifo, O_RDONLY);
    char message[message_size];

    while (true) {
        if (read(fd, message, message_size) > 0) {
            const auto [x, y, color] = decode_message(message);
            std::cout << "x: " << x <<  " y: " << y << " color: " << color << "\n";
        }
    }

    close(fd);
    return 0;
}