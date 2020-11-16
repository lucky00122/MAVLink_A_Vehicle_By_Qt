/*
 * File name: mavlink_udp.h
 * Purpose: Main application for simulating a vehicle which connects to QGC via MAVLink over UDP socket.
 * Creating Date: 2019.12.04
 * Author/Charge: Panda Wang (lucky00122@gmail.com)
 * Note: N/A
 */

#include <QCoreApplication>
#include "mavlink_udp.h"

int main(int argc, char *argv[])
{
	mavlink_udp mavlink;
		
    Q_UNUSED(argc)
    Q_UNUSED(argv)
    QCoreApplication a(argc, argv);

	mavlink.udpInit();

    return a.exec();
}


