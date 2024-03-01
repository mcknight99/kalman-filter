compileRunMain:	
	g++ kf-2d.cpp -c -Wall -o "KF-2D.o"
	g++ main.cpp -Wall -o "Main.exe" Matrix.o Fusion.o ApogeePrediction.o FlightStatus.o KF-2D.o
	.\Main.exe

all:
	@echo ##############################################################################################################################
	g++ Adafruit_AHRS_NXPmatrix.c -c -Wall -o "Matrix.o"
	g++ Adafruit_AHRS_NXPFusion.cpp -c -Wall -o "Fusion.o"
	g++ apogeeprediction.cpp -c -Wall -o "ApogeePrediction.o"
	g++ flightstatus.cpp -c -Wall -o "FlightStatus.o"
	g++ kf-2d.cpp -c -Wall -o "KF-2D.o"
	g++ main.cpp -Wall -o "Main.exe" Matrix.o Fusion.o ApogeePrediction.o FlightStatus.o KF-2D.o

run:
	.\Main.exe

clean:
	del *.o

exe:
	make compileAll
	make clean
	make run