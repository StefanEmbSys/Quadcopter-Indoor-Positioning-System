/*
*  @file    IPS_Output.cpp
*  @brief   Common SW for both methods (TOA and TWR) to support a graphical output in Matlab
*  @author 	Stefan Koller, BSc
*/

#include "pch.h"
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <zmq.h>
#include <engine.h>

int main()
{
	char buffer[32];
	void *context;
	void *replyer;
	mxArray *x = NULL;
	mxArray *y = NULL;
	mxArray *z = NULL;
	context = zmq_ctx_new();
	replyer = zmq_socket(context, ZMQ_REP);
	zmq_bind(replyer, "tcp://192.168.137.1:6000");
	std::cout << "ZMQ bounded in Replyer mode. \n" << std::endl;

	/*
	* Start the MATLAB engine
	*/
	std::cout << "Matlab Engine in startup... \n" << std::endl;
	Engine *ep;
	if (!(ep = engOpen(NULL))) 
	{
		exit(-1);
	}
	std::cout << "Matlab Engine started sucessfully. \n" << std::endl;
	
	// Change folder of Matlab to where the .m script is available
	std::cout << "Change dictionary to where the script is. \n" << std::endl;
	engEvalString(ep, "cd C:\\work\\MasterThesis\\IPS_Output\\IPS_Output\\Matlab");

	while (1)
	{
		int i = 0;
		std::cout << "Waiting for receiption!\n";
		zmq_recv(replyer, (void *)buffer, 32, 0);
		zmq_send(replyer, "0", 1, 0);

		/*
		 * Create variables from buffer
		 */
		x = mxCreateDoubleMatrix(1, 1, mxREAL);
		memcpy((char *)mxGetPr(x), (char *)(buffer + 8), sizeof(double));

		y = mxCreateDoubleMatrix(1, 1, mxREAL);
		memcpy((char *)mxGetPr(y), (char *)(buffer + 16), sizeof(double));

		z = mxCreateDoubleMatrix(1, 1, mxREAL);
		memcpy((char *)mxGetPr(z), (char *)(buffer + 24), sizeof(double));

		/*
		 * Place the variables into the MATLAB workspace
		 */
		engPutVariable(ep, "x", x);
		engPutVariable(ep, "y", y);
		engPutVariable(ep, "z", z);
		engEvalString(ep, "IPS");
	}
}

// Programm ausführen: STRG+F5 oder "Debuggen" > Menü "Ohne Debuggen starten"
// Programm debuggen: F5 oder "Debuggen" > Menü "Debuggen starten"

// Tipps für den Einstieg: 
//   1. Verwenden Sie das Projektmappen-Explorer-Fenster zum Hinzufügen/Verwalten von Dateien.
//   2. Verwenden Sie das Team Explorer-Fenster zum Herstellen einer Verbindung mit der Quellcodeverwaltung.
//   3. Verwenden Sie das Ausgabefenster, um die Buildausgabe und andere Nachrichten anzuzeigen.
//   4. Verwenden Sie das Fenster "Fehlerliste", um Fehler anzuzeigen.
//   5. Wechseln Sie zu "Projekt" > "Neues Element hinzufügen", um neue Codedateien zu erstellen, bzw. zu "Projekt" > "Vorhandenes Element hinzufügen", um dem Projekt vorhandene Codedateien hinzuzufügen.
//   6. Um dieses Projekt später erneut zu öffnen, wechseln Sie zu "Datei" > "Öffnen" > "Projekt", und wählen Sie die SLN-Datei aus.
