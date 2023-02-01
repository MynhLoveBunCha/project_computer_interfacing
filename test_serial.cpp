#include <iostream>
#include <string>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

int main(){
    // open serial port
    int serial_port = open("/dev/ttyUSB0", O_RDWR);

    // create termios struct
    struct termios tty;

    // read in existing setting and handle any error
    if(serial_port < 0){
        cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
        return -1;
    }
    tcgetattr(serial_port, &tty);

    // Setting
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication 
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte 
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;  // Disable canonical mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 50;    // Wait for up to 5s (50 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    tcflush(serial_port, TCIOFLUSH);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    for(int i = 0; i < 5; i++){

        // Allocate memory for write buffer
        char write_buf[256];
        
        // Write to serial port
        if(i % 2 == 0){
            strcpy(write_buf, "1");
        }
        else{
            strcpy(write_buf, "0");
        }
        int len = strlen(write_buf);
        cout << "Wrote " << write(serial_port, &write_buf, len) << " bytes over UART" << endl;

        // Allocate memory for read buffer
        char read_buf [256];
        memset(&read_buf, '\0', sizeof(read_buf));

        // read
        int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

        // check if any error
        if (num_bytes < 0) {
            printf("Error reading: %s", strerror(errno));
            return -1;
        }

        printf("Read %i bytes. Received message: %s\n", num_bytes, read_buf);
    
    }

    close(serial_port);
    return 0;
}