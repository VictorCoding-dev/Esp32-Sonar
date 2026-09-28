#include <Arduino.h>
#include <ESP32Servo.h>

constexpr uint8_t PIN_SERVO_PAN = 18;
constexpr uint8_t PIN_SERVO_TILT = 25;
constexpr uint8_t PIN_ULTRASONIC_TRIG = 19;
constexpr uint8_t PIN_ULTRASONIC_ECHO = 23;
constexpr uint8_t PIN_LASER = 26;
constexpr uint8_t FIRMATA_PAN_CHANNEL = 9;
constexpr uint8_t FIRMATA_TILT_CHANNEL = 10;
constexpr uint8_t ANALOG_MESSAGE = 0xE0;
constexpr uint8_t DIGITAL_MESSAGE = 0x90;
constexpr uint8_t START_SYSEX = 0xF0;
constexpr uint8_t END_SYSEX = 0xF7;
constexpr uint8_t SERVO_CONFIG = 0x70;
constexpr uint8_t MAX_SYSEX_DATA = 8;
constexpr uint8_t DISTANCE_ANALOG_CHANNEL = 0;
constexpr unsigned long DISTANCE_INTERVAL_MS = 100;
constexpr unsigned long ECHO_TIMEOUT_US = 30000;

Servo servoPan;
Servo servoTilt;

bool sysexAtivo = false;
uint8_t sysexComando = 0;
uint8_t sysexDados[MAX_SYSEX_DATA];
uint8_t sysexTamanho = 0;
uint8_t comandoAnalogico = 0;
uint8_t bytesAnalogicos[2];
uint8_t indiceAnalogico = 0;
uint8_t comandoDigital = 0;
uint8_t bytesDigitais[2];
uint8_t indiceDigital = 0;
unsigned long ultimaLeituraDistanciaMs = 0;

Servo* servoDoCanal(uint8_t canal) {
	if (canal == FIRMATA_PAN_CHANNEL) {
		return &servoPan;
	}
	if (canal == FIRMATA_TILT_CHANNEL) {
		return &servoTilt;
	}
	return nullptr;
}

void processarSysex() {
	if (sysexComando != SERVO_CONFIG || sysexTamanho != 5) {
		return;
	}

	uint8_t canal = sysexDados[0];
	Servo* servo = servoDoCanal(canal);
	if (servo == nullptr) {
		return;
	}

	int pulsoMinimo = sysexDados[1] | (sysexDados[2] << 7);
	int pulsoMaximo = sysexDados[3] | (sysexDados[4] << 7);
	uint8_t pinoFisico = canal == FIRMATA_PAN_CHANNEL ? PIN_SERVO_PAN : PIN_SERVO_TILT;
	servo->detach();
	servo->attach(pinoFisico, pulsoMinimo, pulsoMaximo);
	servo->write(90);
}

void processarByte(uint8_t byteRecebido) {
	if (sysexAtivo) {
		if (byteRecebido == END_SYSEX) {
			processarSysex();
			sysexAtivo = false;
			sysexTamanho = 0;
			sysexComando = 0;
		} else if (sysexComando == 0) {
			sysexComando = byteRecebido;
		} else if (sysexTamanho < MAX_SYSEX_DATA) {
			sysexDados[sysexTamanho++] = byteRecebido;
	}
		return;
	}

	if (byteRecebido == START_SYSEX) {
		sysexAtivo = true;
		sysexComando = 0;
		sysexTamanho = 0;
		return;
	}

	if (indiceDigital > 0) {
		bytesDigitais[indiceDigital - 1] = byteRecebido;
		if (++indiceDigital == 3) {
			uint8_t porta = comandoDigital & 0x0F;
			uint16_t mascara = bytesDigitais[0] | (bytesDigitais[1] << 7);
			if (porta == (PIN_LASER / 8)) {
				digitalWrite(PIN_LASER, (mascara & (1U << (PIN_LASER % 8))) ? HIGH : LOW);
			}
			indiceDigital = 0;
		}
		return;
	}

	if (indiceAnalogico > 0) {
		bytesAnalogicos[indiceAnalogico - 1] = byteRecebido;
		if (++indiceAnalogico == 3) {
			uint8_t canal = comandoAnalogico & 0x0F;
			int angulo = bytesAnalogicos[0] | (bytesAnalogicos[1] << 7);
			Servo* servo = servoDoCanal(canal);
			if (servo != nullptr) {
				servo->write(constrain(angulo, 0, 180));
			}
			indiceAnalogico = 0;
		}
		return;
	}

	if ((byteRecebido & 0xF0) == ANALOG_MESSAGE) {
		comandoAnalogico = byteRecebido;
		indiceAnalogico = 1;
	} else if ((byteRecebido & 0xF0) == DIGITAL_MESSAGE) {
		comandoDigital = byteRecebido;
		indiceDigital = 1;
	}
}

void enviarDistanciaFirmata() {
	digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
	delayMicroseconds(2);
	digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
	delayMicroseconds(10);
	digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

	unsigned long duracao = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, ECHO_TIMEOUT_US);
	uint16_t valorAnalogico = 0;
	if (duracao > 0) {
		float distanciaCm = (duracao * 0.0343f) / 2.0f;
		valorAnalogico = constrain(static_cast<int>(lroundf(distanciaCm)) + 1, 1, 401);
	}

	Serial.write(ANALOG_MESSAGE | DISTANCE_ANALOG_CHANNEL);
	Serial.write(valorAnalogico & 0x7F);
	Serial.write((valorAnalogico >> 7) & 0x7F);
}

void setup() {
	Serial.begin(57600);
	pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
	pinMode(PIN_ULTRASONIC_ECHO, INPUT);
	pinMode(PIN_LASER, OUTPUT);
	digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
	digitalWrite(PIN_LASER, LOW);
	servoPan.setPeriodHertz(50);
	servoTilt.setPeriodHertz(50);
	servoPan.attach(PIN_SERVO_PAN, 544, 2400);
	servoTilt.attach(PIN_SERVO_TILT, 544, 2400);
	servoPan.write(90);
	servoTilt.write(90);
}

void loop() {
	while (Serial.available() > 0) {
		processarByte(static_cast<uint8_t>(Serial.read()));
	}

	if (millis() - ultimaLeituraDistanciaMs >= DISTANCE_INTERVAL_MS) {
		ultimaLeituraDistanciaMs = millis();
		enviarDistanciaFirmata();
	}
}
