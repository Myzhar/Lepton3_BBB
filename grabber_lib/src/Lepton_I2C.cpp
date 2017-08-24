#include <stdio.h>
#include "Lepton_I2C.h"
#include "LEPTON_SDK.h"
#include "LEPTON_SYS.h"
#include "LEPTON_Types.h"
#include "LEPTON_AGC.h"
#include "LEPTON_RAD.h"
bool _connected;

LEP_CAMERA_PORT_DESC_T _port;
LEP_SYS_FPA_TEMPERATURE_KELVIN_T fpa_temp_kelvin;
LEP_RESULT result;

int lepton_connect() 
{
	result = LEP_OpenPort(1, LEP_CCI_TWI, 400, &_port);
	
	if (result != LEP_OK) 
	{
    	return -1;
  	}
  	
	_connected = true;
	return 0;
}

int lepton_temperature()
{
	if(!_connected)
		lepton_connect();
	result = ((LEP_GetSysFpaTemperatureKelvin(&_port, &fpa_temp_kelvin)));
	printf("FPA temp kelvin: %i, code %i\n", fpa_temp_kelvin, result);
	return (fpa_temp_kelvin/100);
}


float raw2Celsius(float raw)
{
	float ambientTemperature = 25.0;
	float slope = 0.0217;
	return slope*raw+ambientTemperature-177.77;
}

void lepton_perform_ffc() 
{
	if(!_connected) 
	{
		lepton_connect();
	}
	LEP_RunSysFFCNormalization(&_port);
}

int enable_radiometry( bool enable )
{
	if(!_connected) 
	{
		lepton_connect();
	}
	
	LEP_RAD_ENABLE_E_PTR rad_status;
	
	if( LEP_GetRadEnableState(&_port, rad_status ) != LEP_OK )
		return -1;
	
	LEP_RAD_ENABLE_E new_status = enable?LEP_RAD_ENABLE:LEP_RAD_DISABLE;
	
	if( *rad_status != new_status )
	{
		if( LEP_SetRadEnableState(&_port, new_status ) != LEP_OK )
			return -1;
	}
	
	return new_status;
}

