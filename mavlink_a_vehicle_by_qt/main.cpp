// MAVLink Vehicle Server

#include <QCoreApplication>

#include "mavlink_udp.h"

int main(int argc, char *argv[])
{
	mavlink_udp mavlink;
		
	Q_UNUSED(argc);
	Q_UNUSED(argv);
    QCoreApplication a(argc, argv);

	mavlink.udpInit();

    return a.exec();
}


