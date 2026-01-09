#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>

#include "agent_curas.h"
#include "imageprocessor.h"

namespace Ui {
class MainWindow;
}


class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
private:
    void setSingleMode(int _mode, bool _enable);
    void setSingleMode(bool _enable);
    void setAutoMode(bool _enable);
    void setSemiAutoMode(int _mode, bool _enable);
    void setSemiAutoMode(bool _enable);
    void setMultipleMode(int _mode, bool _enable);
    void setMultipleMode(bool _enable);

    void setOpMode(bool _enable);

    void setLowTemp(bool _enable);
    void setMidTemp(bool _enable);
    void setHighTemp(bool _enable);

signals:

private slots:
    void setReady();
    void setStandby();
    void setShotmode(int);
    void setTargetWait(void);
    void setTemperatureState(void);
    /* ========== EEPROM Tip JYH Append ========================================  */
    void setTipState(void);
    /* ========== EEPROM Tip JYH Append ========================================  */


    void SLT_displayImage(unsigned _width, unsigned _height, unsigned char* _data);

    void on_spb_set_blue_valueChanged(double arg1);
    void on_spb_set_red_valueChanged(double arg1);
    void on_spb_set_green_valueChanged(double arg1);
    void on_spb_set_brightness_valueChanged(int arg1);
    void on_spb_set_contrast_valueChanged(int arg1);
    void on_spb_set_usat_valueChanged(int arg1);
    void on_spb_set_vsat_valueChanged(int arg1);
    void on_btn_reset_clicked();

    void on_spb_set_gain_valueChanged(double arg1);

    void on_spb_set_vsat_2_valueChanged(int arg1);

    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;

    AgentCuRAS* agentCuRAS;
    ImageProcessor* imageProcessor;
};

#endif // MAINWINDOW_H
