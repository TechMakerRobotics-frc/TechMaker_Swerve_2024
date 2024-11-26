package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedState;

public class LedsControl extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;
  private int rainbowFirstPixelHue;
  private int currentPixel = 0; // Para efeitos em sequência

  public LedsControl() {
    // Inicializa o LED no canal 0
    led = new AddressableLED(1);

    // Cria um buffer de 60 LEDs
    ledBuffer = new AddressableLEDBuffer(60);

    // Define o comprimento do LED
    led.setLength(ledBuffer.getLength());

    // Define os dados iniciais e inicia os LEDs
    led.setData(ledBuffer);
    led.start();

    // Inicializa a variável para o arco-íris
    rainbowFirstPixelHue = 0;
  }

  @Override
  public void periodic() {
    // Atualiza os LEDs a cada ciclo (exemplo: arco-íris)
    led.setData(ledBuffer);
    rainbow();
  }

  /** Método para criar o efeito arco-íris */
  public void rainbow() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }

  /** Define todos os LEDs para uma cor vermelha */
  public void setAllRed() {
    setAllColorHSV(0, 255, 255); // Hue 0 = vermelho
  }

  /** Define todos os LEDs para uma cor verde */
  public void setAllGreen() {
    setAllColorHSV(120, 255, 255); // Hue 120 = verde
  }

  /** Define todos os LEDs para uma cor azul */
  public void setAllBlue() {
    setAllColorHSV(240, 255, 255); // Hue 240 = azul
  }

  /** Define todos os LEDs para uma cor amarela */
  public void setAllYellow() {
    setAllColorHSV(60, 255, 255); // Hue 60 = amarelo
  }

  /** Define todos os LEDs para uma cor ciano */
  public void setAllCyan() {
    setAllColorHSV(180, 255, 255); // Hue 180 = ciano
  }

  /** Define todos os LEDs para uma cor magenta */
  public void setAllMagenta() {
    setAllColorHSV(300, 255, 255); // Hue 300 = magenta
  }

  /** Define todos os LEDs para uma cor RGB */
  public void setAllColorRGB(int red, int green, int blue) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, red, green, blue);
    }
    led.setData(ledBuffer);
  }

  /** Define todos os LEDs para uma cor HSV */
  public void setAllColorHSV(int hue, int saturation, int value) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, hue, saturation, value);
    }
    led.setData(ledBuffer);
  }

  /** Efeito: Acender LEDs em sequência */
  public void chaseEffect(int red, int green, int blue) {
    // Apaga todos os LEDs primeiro
    setAllColorRGB(0, 0, 0);

    // Acende o próximo LED
    ledBuffer.setRGB(currentPixel, red, green, blue);

    // Incrementa para o próximo LED
    currentPixel = (currentPixel + 1) % ledBuffer.getLength();

    // Atualiza os LEDs
    led.setData(ledBuffer);
  }

  /** Efeito: Piscar todos os LEDs */
  public void blinkEffect(int red, int green, int blue, boolean isOn) {
    if (isOn) {
      setAllColorRGB(red, green, blue); // Liga com a cor especificada
    } else {
      setAllColorRGB(0, 0, 0); // Desliga
    }
  }

  /** Efeito: Gradiente de uma cor para outra */
  public void gradientEffect(
      int redStart, int greenStart, int blueStart, int redEnd, int greenEnd, int blueEnd) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      double ratio = (double) i / (ledBuffer.getLength() - 1);
      int red = (int) (redStart + ratio * (redEnd - redStart));
      int green = (int) (greenStart + ratio * (greenEnd - greenStart));
      int blue = (int) (blueStart + ratio * (blueEnd - blueStart));
      ledBuffer.setRGB(i, red, green, blue);
    }
    led.setData(ledBuffer);
  }

  public void setState(LedState state) {
    switch (state) {
      case RED:
        setAllRed();
        break;
      case GREEN:
        setAllGreen();
        break;
      case BLUE:
        setAllBlue();
        break;
      case YELLOW:
        setAllYellow();
        break;
      case CIAN:
        setAllCyan();
        break;
      case MAGENTA:
        setAllMagenta();
        break;
      case RAINBOW:
        rainbow();
        break;
    }
  }
}
