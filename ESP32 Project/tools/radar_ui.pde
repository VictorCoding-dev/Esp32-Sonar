import processing.serial.*;

Serial porta;
final String PORTA_SERIAL = "COM15";
final int BAUD_RATE = 115200;
final int DISTANCIA_MAXIMA_CM = 400;

int angulo = 90;
int distancia = DISTANCIA_MAXIMA_CM;
String ultimaLinha = "Aguardando dados...";
String bufferSerial = "";
float mpuRoll = 0;
float mpuPitch = 0;
float mpuTemperatura = 0;
boolean mpuAlerta = false;
boolean mpuDisponivel = false;
int ultimoPacoteMpuMs = 0;
PFont fonte;

void setup() {
  size(1400, 800);
  smooth(4);
  fonte = createFont("Consolas", 20);

  println("Portas seriais disponiveis:");
  println(Serial.list());
  porta = new Serial(this, PORTA_SERIAL, BAUD_RATE);
  porta.clear();
}

void draw() {
  lerDadosSeriais();
  background(4, 10, 18);
  drawRadar();
  drawSweep();
  drawObject();
  drawInfo();
  drawMpuPanel();
}

void lerDadosSeriais() {
  while (porta.available() > 0) {
    char caractere = porta.readChar();
    if (caractere == '\r') {
      continue;
    }

    if (caractere == '\n') {
      processarLinhaMpu(bufferSerial);
      bufferSerial = "";
    } else if (caractere == '.') {
      if (bufferSerial.indexOf("Roll:") >= 0) {
        bufferSerial += caractere;
      } else {
        processarPacoteServo(bufferSerial);
        bufferSerial = "";
      }
    } else if (bufferSerial.length() < 256) {
      bufferSerial += caractere;
    } else {
      bufferSerial = "";
    }
  }
}

void processarPacoteServo(String pacote) {
  pacote = trim(pacote);
  String[] valores = split(pacote, ',');
  if (valores.length != 2) {
    return;
  }

  try {
    angulo = constrain(int(valores[0]), 0, 180);
    distancia = constrain(int(valores[1]), 0, DISTANCIA_MAXIMA_CM);
    ultimaLinha = angulo + "," + distancia;
  } catch (Exception erro) {
    println("Pacote invalido: " + pacote);
  }
}

void processarLinhaMpu(String linha) {
  String[] valores = match(trim(linha),
    "Roll:\\s*([-+0-9.]+)\\s*\\|\\s*Pitch:\\s*([-+0-9.]+)\\s*\\|\\s*Temp:\\s*([-+0-9.]+)\\s*C\\s*\\|\\s*Alerta:\\s*(SIM|NAO)");
  if (valores == null || valores.length < 5) {
    return;
  }

  try {
    mpuRoll = float(valores[1]);
    mpuPitch = float(valores[2]);
    mpuTemperatura = float(valores[3]);
    mpuAlerta = valores[4].equals("SIM");
    mpuDisponivel = true;
    ultimoPacoteMpuMs = millis();
  } catch (Exception erro) {
    println("Linha MPU invalida: " + linha);
  }
}

void drawRadar() {
  pushMatrix();
  translate(width / 2, height - 80);
  noFill();
  stroke(32, 210, 120);
  strokeWeight(2);

  for (int raio = 180; raio <= 720; raio += 180) {
    arc(0, 0, raio * 2, raio * 2, PI, TWO_PI);
  }

  line(-720, 0, 720, 0);
  for (int marca = 0; marca <= 180; marca += 30) {
    line(0, 0, 720 * cos(radians(180 - marca)), -720 * sin(radians(marca)));
  }
  popMatrix();
}

void drawSweep() {
  pushMatrix();
  translate(width / 2, height - 80);
  stroke(80, 255, 150);
  strokeWeight(4);
  line(0, 0, 720 * cos(radians(180 - angulo)), -720 * sin(radians(angulo)));
  popMatrix();
}

void drawObject() {
  if (distancia >= DISTANCIA_MAXIMA_CM) {
    return;
  }

  pushMatrix();
  translate(width / 2, height - 80);
  float raio = map(distancia, 0, DISTANCIA_MAXIMA_CM, 0, 720);
  stroke(255, 65, 65);
  strokeWeight(12);
  line(raio * cos(radians(180 - angulo)), -raio * sin(radians(angulo)),
       720 * cos(radians(180 - angulo)), -720 * sin(radians(angulo)));
  popMatrix();
}

void drawInfo() {
  fill(180, 255, 210);
  textFont(fonte);
  text("ESP32 RADAR", 36, 48);
  text("Porta: " + PORTA_SERIAL + "   Baud: " + BAUD_RATE, 36, 80);
  text("Angulo: " + angulo + " graus", 36, 124);
  text("Distancia: " + distancia + " cm", 36, 158);
  text("Pacote: " + ultimaLinha, 36, 192);

  if (distancia < 40) {
    fill(255, 80, 80);
    text("OBJETO DETECTADO", 36, 236);
  } else {
    fill(120, 220, 160);
    text("AREA LIVRE", 36, 236);
  }

  fill(110, 180, 140);
  text("GPIO PAN 21 | TILT 22 | TRIG 19 | ECHO 23", 36, height - 28);
}

void drawMpuPanel() {
  float painelX = width - 330;
  float painelY = 24;
  float painelW = 300;
  float painelH = 190;

  noStroke();
  fill(4, 10, 18, 235);
  rect(painelX, painelY, painelW, painelH, 8);
  noFill();
  stroke(65, 103, 110);
  strokeWeight(1);
  rect(painelX, painelY, painelW, painelH, 8);

  textFont(fonte);
  fill(180, 255, 210);
  text("MPU REMOTO / ESP-NOW", painelX + 16, painelY + 28);
  stroke(32, 110, 105);
  line(painelX + 16, painelY + 40, painelX + painelW - 16, painelY + 40);

  textSize(16);
  if (!mpuDisponivel) {
    fill(150, 170, 170);
    text("Aguardando dados MPU...", painelX + 16, painelY + 72);
    return;
  }

  textFont(fonte);
  fill(180, 255, 210);
  text("Roll: " + nf(mpuRoll, 1, 1) + " deg", painelX + 16, painelY + 70);
  text("Pitch: " + nf(mpuPitch, 1, 1) + " deg", painelX + 16, painelY + 101);
  text("Temp: " + nf(mpuTemperatura, 1, 1) + " C", painelX + 16, painelY + 132);

  boolean sinalAtivo = millis() - ultimoPacoteMpuMs < 1500;
  if (mpuAlerta) {
    fill(255, 90, 80);
    text("ALERTA MPU ATIVO", painelX + 16, painelY + 166);
  } else {
    fill(sinalAtivo ? color(120, 220, 160) : color(255, 170, 80));
    text(sinalAtivo ? "SINAL MPU ATIVO" : "SEM DADOS RECENTES", painelX + 16, painelY + 166);
  }
}
