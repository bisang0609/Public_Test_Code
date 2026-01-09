#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPixmap>
#include <QPainter>


#define __DEVELOPMENT__


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    agentCuRAS(new AgentCuRAS)
{
    ui->setupUi(this);

    ui->lbl_options_value->setVisible(false);

    imageProcessor = new ImageProcessor(agentCuRAS);

    ui->OSD->setCuRAS(agentCuRAS);

#if 0   // 20221007 JYH Append
    connect(imageProcessor, SIGNAL(SIG_setImage(unsigned, unsigned, unsigned char*)), this, SLOT(SLT_displayImage(unsigned, unsigned, unsigned char*)), Qt::DirectConnection);
#else
    connect(imageProcessor, SIGNAL(SIG_setImage(unsigned, unsigned, unsigned char*)), this, SLOT(SLT_displayImage(unsigned, unsigned, unsigned char*)), Qt::QueuedConnection);  //Qt::AutoConnection);
#endif   // 20221007 JYH Append

    connect(imageProcessor, SIGNAL(drawCross(cv::Point)), ui->OSD, SLOT(drawCross(cv::Point)));
    connect(imageProcessor, SIGNAL(drawSingleGuide(cv::Point)), ui->OSD, SLOT(drawSingleGuide(cv::Point)));
    connect(imageProcessor, SIGNAL(drawMultipleGuide(std::vector<cv::Point>)), ui->OSD, SLOT(drawMultipleGuide(std::vector<cv::Point>)));
    connect(imageProcessor, SIGNAL(drawMultipleGuide(std::vector<cv::Point>, int)), ui->OSD, SLOT(drawMultipleGuideEx(std::vector<cv::Point>, int)));

    connect(agentCuRAS, SIGNAL(setShotOk()), ui->OSD, SLOT(drawShotOk()));
    connect(agentCuRAS, SIGNAL(setShotOk(uint8_t)), ui->OSD, SLOT(drawShotOk(uint8_t)));

    connect(agentCuRAS, SIGNAL(SIG_setReady()), this, SLOT(setReady()));
    connect(agentCuRAS, SIGNAL(SIG_setStandby()), this, SLOT(setStandby()));
    connect(agentCuRAS, SIGNAL(SIG_setShotmode(int)), this, SLOT(setShotmode(int)));
    connect(agentCuRAS, SIGNAL(SIG_setTargetWait()), this, SLOT(setTargetWait()));
    connect(agentCuRAS, SIGNAL(SIG_setTemperatureState()), this, SLOT(setTemperatureState()));
    /* ========== EEPROM Tip JYH Append ========================================  */
    connect(agentCuRAS, SIGNAL(SIG_setTipState()), this, SLOT(setTipState()));
    ui->lbl_tip_progress->setVisible(false);
    ui->lbl_EEPROMConnectTXT->setVisible(false);
    /* ========== EEPROM Tip JYH Append ========================================  */

    imageProcessor->start();
}


MainWindow::~MainWindow()
{
    delete agentCuRAS;
    delete ui;
}



void MainWindow::setReady()
{
    static QPixmap state_ready(":/images/pc_btn_o.png");
    static QPixmap state_check(":/images/pc_btn_chceck.png");

    if (agentCuRAS->isTargetWait())
        ui->lbl_state->setPixmap(state_check);
    else
        ui->lbl_state->setPixmap(state_ready);
}

void MainWindow::setStandby()
{
    static QPixmap state_standby(":/images/pc_btn_n.png");
    static QPixmap blank_stream(":/images/img_pc_logo.pn");

    ui->lbl_state->setPixmap(state_standby);
    ui->lbl_main_stream->clear();

    ui->OSD->clear();
}

#if 1   //20220622 JYH EEPROM GUI 추가
#define MANUAL_LABEL0()  ui->lbl_mode_0->move(1300, 356)
#define MANUAL_LABEL1()  ui->lbl_mode_1->move(1400, 356)
#else
#define MANUAL_LABEL0()  ui->lbl_mode_0->move(1300, 398)
#define MANUAL_LABEL1()  ui->lbl_mode_1->move(1400, 398)
#endif
void MainWindow::setSingleMode(int _mode, bool _enable)
{
    static QPixmap main_single(":/images/btn_treat_mode_1.png");
    static QPixmap mode_single_o(":/images/btn_treat_mode1_o.png");
    static QPixmap mode_single_n(":/images/btn_treat_mode1_n.png");
    static QPixmap mode_single_options(":/images/pc_single_prr.png");

    if (_enable)
    {
        setMultipleMode(false);

#if 1       // 20240826 JYH Modify
        if (agentCuRAS->getUseMultiShotMode() == 0)
        {
            ui->lbl_mode_0->move(1400, 356);
            ui->lbl_mode_1->hide();
            ui->lbl_mode_2->hide();
        }
        else {
            ui->lbl_mode_1->show();
            ui->lbl_mode_2->show();
            MANUAL_LABEL0();
            MANUAL_LABEL1();
        }
#else
        MANUAL_LABEL0();
        MANUAL_LABEL1();
#endif

        ui->lbl_main_shotmode->setPixmap(main_single);
        ui->lbl_mode_0->setPixmap(mode_single_o);
        ui->lbl_options->setPixmap(mode_single_options);
        switch (_mode)
        {
        case SINGLE_NONE:
            ui->lbl_options_value->setText("None");
            break;
        case SINGLE_1HZ:
            ui->lbl_options_value->setText("1");
            break;
        case SINGLE_2HZ:
            ui->lbl_options_value->setText("2");
            break;
        }
    }
    else
    {
        ui->lbl_mode_0->setPixmap(mode_single_n);
    }
}

void MainWindow::setSingleMode(bool _enable)
{
    setSingleMode(NONE, _enable);
}


#if 1   //20220622 JYH EEPROM GUI 추가
#define AUTO_LABEL0()  ui->lbl_mode_0->move(1340, 356)
#define AUTO_LABEL1()  ui->lbl_mode_1->move(1460, 356)
#else
#define AUTO_LABEL0()  ui->lbl_mode_0->move(1340, 398)
#define AUTO_LABEL1()  ui->lbl_mode_1->move(1460, 398)
#endif

void MainWindow::setAutoMode(bool _enable)
{
    static QPixmap main_auto(":/images/btn_treat_mode_2.png");
    static QPixmap mode_auto_n(":/images/btn_treat_mode2_n.png");
    static QPixmap mode_auto_o(":/images/btn_treat_mode2_o.png");

    if (_enable)
    {
        setSemiAutoMode(false);

        AUTO_LABEL0();
        AUTO_LABEL1();

        ui->lbl_main_shotmode->setPixmap(main_auto);
        ui->lbl_mode_0->setPixmap(mode_auto_o);
    }
    else
    {
        ui->lbl_mode_0->setPixmap(mode_auto_n);
    }
}

void MainWindow::setSemiAutoMode(int _mode, bool _enable)
{
    static QPixmap main_semigv(":/images/btn_treat_mode_3.png");
    static QPixmap main_semiwh(":/images/btn_treat_mode_3.png");

    static QPixmap mode_semi_n(":/images/btn_treat_mode3_n.png");
    static QPixmap mode_semi_o(":/images/btn_treat_mode3_o.png");

    static QPixmap color_n(":/images/pc_gv_white_n.PNG");
    static QPixmap color_gv(":/images/pc_gv_on.png");
    static QPixmap color_wh(":/images/pc_white_on.png");

    if (_enable)
    {
        setAutoMode(false);

        AUTO_LABEL0();
        AUTO_LABEL1();

        switch (_mode)
        {
        case SEMIAUTO_GV:
            ui->lbl_main_shotmode->setPixmap(main_semigv);
            ui->lbl_mode_1->setPixmap(mode_semi_o);
            //ui->lbl_options->setPixmap(color_gv);
            break;
        case SEMIAUTO_WHITE:
            ui->lbl_main_shotmode->setPixmap(main_semiwh);
            ui->lbl_mode_1->setPixmap(mode_semi_o);
            //ui->lbl_options->setPixmap(color_wh);
            break;
        }
    }
    else
    {
        ui->lbl_mode_1->setPixmap(mode_semi_n);
        ui->lbl_options->setPixmap(color_n);
    }
}

void MainWindow::setSemiAutoMode(bool _enable)
{
    setSemiAutoMode(NONE, _enable);
}

void MainWindow::setMultipleMode(int _mode, bool _enable)
{
    static QPixmap main_multiple(":/images/btn_treat_mode_4.png");
    static QPixmap mode_multiple_n(":/images/btn_treat_mode4_n.png");
    static QPixmap mode_multiple_o(":/images/btn_treat_mode4_o.png");
    static QPixmap mode_multiple_options(":/images/pc_multiple_area.png");
    static QPixmap main_multiple_20MM_square(":/images/btn_treat_mode_5.png");
    static QPixmap mode_multiple_20MM_square_n(":/images/btn_treat_mode5_n.png");
    static QPixmap mode_multiple_20MM_square_o(":/images/btn_treat_mode5_o.png");

    if (_enable)
    {
        setSemiAutoMode(false);
        setAutoMode(false);
        setSingleMode(false);

        MANUAL_LABEL0();
        MANUAL_LABEL1();

        if (_mode != MULTIPLE_SQUARE_20MM )
        {
            ui->lbl_main_shotmode->setPixmap(main_multiple);
            ui->lbl_mode_1->setPixmap(mode_multiple_o);
            ui->lbl_mode_2->setPixmap(mode_multiple_20MM_square_n);
        }
        else
        {
            ui->lbl_main_shotmode->setPixmap(main_multiple_20MM_square);
            ui->lbl_mode_1->setPixmap(mode_multiple_n);
            ui->lbl_mode_2->setPixmap(mode_multiple_20MM_square_o);
        }

        ui->lbl_options->setPixmap(mode_multiple_options);
        switch (_mode)
        {
        case MULTIPLE_6MM:
            ui->lbl_options_value->setText("6");
            break;
        case MULTIPLE_10MM:
            ui->lbl_options_value->setText("10");
            break;
        case MULTIPLE_15MM:
            ui->lbl_options_value->setText("15");
            break;
        case MULTIPLE_20MM:
            ui->lbl_options_value->setText("20");
            break;
        case MULTIPLE_SQUARE_20MM:
            ui->lbl_options_value->setText("20");
            break;
        }

    }
    else
    {
        ui->lbl_mode_1->setPixmap(mode_multiple_n);
        ui->lbl_mode_2->setPixmap(mode_multiple_20MM_square_n);
    }
}

void MainWindow::setMultipleMode(bool _enable)
{
    setMultipleMode(NONE, _enable);
}

enum {
    OP_MANUAL = false,
    OP_AUTO = true
};

void MainWindow::setOpMode(bool _enable)
{
    static QPixmap op_mode_auto(":/images/pc_mode_a.png");
    static QPixmap op_mode_manual(":/images/pc_mode_m.png");

    if (_enable)
    {
        ui->lbl_op_mode->setPixmap(op_mode_auto);

        ui->lbl_mode_1->show(); // 20240826 JYH Modify
        ui->lbl_mode_2->hide();
        ui->lbl_options->hide();
        ui->lbl_options_value->hide();
    }
    else
    {
        ui->lbl_op_mode->setPixmap(op_mode_manual);
#if 1       // 20240826 JYH Modify
        if (agentCuRAS->getUseMultiShotMode() == 0)
        {
            ui->lbl_mode_1->hide();
            ui->lbl_mode_2->hide();
        }
        else {
            ui->lbl_mode_1->show();
            ui->lbl_mode_2->show();
        }
#else
        ui->lbl_mode_2->show();
#endif
        ui->lbl_options->show();
        ui->lbl_options_value->show();
    }
}

void MainWindow::setShotmode(int _mode)
{
   switch (_mode)
    {
    case SINGLE_NONE:
    case SINGLE_1HZ:
    case SINGLE_2HZ:
        setOpMode(OP_MANUAL);
        setSingleMode(_mode, true);
        break;
    case MULTIPLE_6MM:
    case MULTIPLE_10MM:
    case MULTIPLE_15MM:
    case MULTIPLE_20MM:
    case MULTIPLE_SQUARE_20MM:
        setOpMode(OP_MANUAL);
        setMultipleMode(_mode, true);
        break;

    case AUTO:
        setOpMode(OP_AUTO);
        setAutoMode(true);
        break;

    case SEMIAUTO_GV:
    case SEMIAUTO_WHITE:
        setOpMode(OP_AUTO);
        setSemiAutoMode(_mode, true);
        break;

    }
}

void MainWindow::setLowTemp(bool _enable)
{
    static QPixmap icon_temp_low_normal(":/images/btn_treat_tec_l_1.png");
    static QPixmap icon_temp_low_blue(":/images/btn_treat_tec_l_2.png");
    static QPixmap label_temp_low_off(":/images/btn_treat_tec1_n.png");
    static QPixmap label_temp_low_on(":/images/btn_treat_tec1_o.png");

    if (_enable)
    {
        if (agentCuRAS->isTargetOk())
            ui->lbl_icon_temp->setPixmap(icon_temp_low_blue);
        else
            ui->lbl_icon_temp->setPixmap(icon_temp_low_normal);
        ui->lbl_temp_low->setPixmap(label_temp_low_on);
    }
    else
    {
        ui->lbl_temp_low->setPixmap(label_temp_low_off);
    }
}

void MainWindow::setMidTemp(bool _enable)
{
    static QPixmap icon_temp_mid_normal(":/images/btn_treat_tec_m_1.png");
    static QPixmap icon_temp_mid_blue(":/images/btn_treat_tec_m_2.png");
    static QPixmap label_temp_mid_off(":/images/btn_treat_tec2_n.png");
    static QPixmap label_temp_mid_on(":/images/btn_treat_tec2_o.png");

    if (_enable)
    {
        if (agentCuRAS->isTargetOk())
            ui->lbl_icon_temp->setPixmap(icon_temp_mid_blue);
        else
            ui->lbl_icon_temp->setPixmap(icon_temp_mid_normal);
        ui->lbl_temp_mid->setPixmap(label_temp_mid_on);
    }
    else
    {
        ui->lbl_temp_mid->setPixmap(label_temp_mid_off);
    }
}

void MainWindow::setHighTemp(bool _enable)
{
    static QPixmap icon_temp_high_normal(":/images/btn_treat_tec_h_1.png");
    static QPixmap icon_temp_high_blue(":/images/btn_treat_tec_h_2.png");
    static QPixmap label_temp_high_off(":/images/btn_treat_tec3_n.png");
    static QPixmap label_temp_high_on(":/images/btn_treat_tec3_o.png");

    if (_enable)
    {
        if (agentCuRAS->isTargetOk())
            ui->lbl_icon_temp->setPixmap(icon_temp_high_blue);
        else
            ui->lbl_icon_temp->setPixmap(icon_temp_high_normal);
        ui->lbl_temp_high->setPixmap(label_temp_high_on);
    }
    else
    {
        ui->lbl_temp_high->setPixmap(label_temp_high_off);
    }
}


void MainWindow::setTargetWait(void)
{
    if (agentCuRAS->getOpMode() == READY)
        setReady();
}


void MainWindow::setTemperatureState(void)
{
    switch (agentCuRAS->getTargetTemp())
    {
    case LOW_TEMPERATURE:
        setMidTemp(false);
        setHighTemp(false);
        setLowTemp(true);
        break;
    case MID_TEMPERATURE:
        setHighTemp(false);
        setLowTemp(false);
        setMidTemp(true);
        break;
    case HIGH_TEMPERATURE:
        setLowTemp(false);
        setMidTemp(false);
        setHighTemp(true);
        break;
    }
}

/* ========== EEPROM Tip JYH Append ========================================  */
void MainWindow::setTipState(void)
{
    static QPixmap icon_tip_new(":/images/img_tip_new.png");
    static QPixmap img_ConnectNewBar(":/images/img_tip_newtbar.png");
    static QPixmap icon_tip_normal(":/images/img_tip_connect.png");
    static QPixmap icon_tip_disable(":/images/img_tip_disable.png");
    static QPixmap icon_tip_reach(":/images/img_tip_reach.png");
    static QPixmap img_ConnectBar(":/images/img_tip_connectbar.png");

    int nPercent = (int)agentCuRAS->getTipState();
    if (agentCuRAS->getEEPROMStatus() == 0xAAAA || agentCuRAS->getEEPROMStatus() == 0x0707 ||
            nPercent > 100 || nPercent < 0)
    {
        nPercent = 100;
    }
    if ((agentCuRAS->getEEPROMStatus() == 0x0101 && nPercent > 0 && nPercent <= 100) ||
             agentCuRAS->getEEPROMStatus() == 0xAAAA || agentCuRAS->getEEPROMStatus() == 0x0707)
    {
        ui->lbl_tip_progress->setVisible(true);
        QSize tipPixSize = img_ConnectBar.size();
        //QSize tipPixSize = ui->lbl_tip_progress->size();

        if (agentCuRAS->getEEPROMStatus() == 0xAAAA)
        {
            //<html><head/><body><p><img src=":/images/img_tip_connectbar.png"/></p></body></html>
            ui->lbl_tip_progress->setText("<html><head/><body><p><img src=\":/images/img_tip_newtbar.png\"/></p></body></html>");
            ui->lbl_tip_progress->resize(tipPixSize.width(), tipPixSize.height());
            ui->lbl_tip_state->setPixmap(icon_tip_new);
            //qDebug("New tip connected");
        }
        else {
            ui->lbl_tip_progress->setText("<html><head/><body><p><img src=\":/images/img_tip_connectbar.png\"/></p></body></html>");
            ui->lbl_tip_progress->resize(tipPixSize.width()*nPercent/100, tipPixSize.height());
            ui->lbl_tip_state->setPixmap(icon_tip_normal);
            //qDebug("Tip connected");
        }

        QPalette palette;
        QBrush brush(QColor(255, 255, 255, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        ui->lbl_EEPROMConnectTXT->setPalette(palette);

        ui->lbl_EEPROMConnectTXT->setVisible(true);
        ui->lbl_EEPROMConnectTXT->setText("Connected");
    }
    else if (agentCuRAS->getEEPROMStatus() == 0x0505)
    {
        //ui->lbl_tip_progress->hide();
        ui->lbl_tip_progress->setVisible(false);
        ui->lbl_tip_state->setPixmap(icon_tip_reach);

        QPalette palette;
        QBrush brush(QColor(255, 255, 255, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        ui->lbl_EEPROMConnectTXT->setPalette(palette);

        ui->lbl_EEPROMConnectTXT->setVisible(true);
        ui->lbl_EEPROMConnectTXT->setText("Time Limit Reached");

        //qDebug("Time Limit Reached");
    }
    else {
        //ui->lbl_tip_progress->hide();
        ui->lbl_tip_progress->setVisible(false);
        ui->lbl_tip_state->setPixmap(icon_tip_disable);

        QPalette palette;
        QBrush brush(QColor(150, 150, 150, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        ui->lbl_EEPROMConnectTXT->setPalette(palette);

        ui->lbl_EEPROMConnectTXT->setVisible(true);
        ui->lbl_EEPROMConnectTXT->setText("Not Connected");

        //qDebug("Not Connected");
    }
    // Tip display Test code
    /*ui->lbl_tip_progress->setVisible(true);
    QSize tipPixSize = img_ConnectBar.size();

    ui->lbl_tip_progress->setText("<html><head/><body><p><img src=\":/images/img_tip_newtbar.png\"/></p></body></html>");
    ui->lbl_tip_progress->resize(tipPixSize.width(), tipPixSize.height());
    ui->lbl_tip_state->setPixmap(icon_tip_new);*/

}
/* ========== EEPROM Tip JYH Append ========================================  */

void MainWindow::SLT_displayImage(unsigned _width, unsigned _height, unsigned char* _data)
{
//    QImage *_image = new QImage(_data, _width , _height, QImage::Format_RGB16);
#if 0   // 20221007 JYH Append
    QImage *_image = new QImage(_data, _width , _height, QImage::Format_RGB888);
#endif   // 20221007 JYH Append

    if(agentCuRAS->getOpMode() == READY)
    {
#if 0   // 20221007 JYH Append
        ui->lbl_main_stream->setScaledContents(true);
        ui->lbl_main_stream->setPixmap(QPixmap::fromImage(*_image));
#else   // 20221007 JYH Append
        int w = ui->lbl_main_stream->width();
        int h = ui->lbl_main_stream->height();

        QImage *_image = new QImage(_data, _width , _height, QImage::Format_RGB888);
        QPixmap _Pixmap = QPixmap::fromImage(*_image);
        ui->lbl_main_stream->setPixmap(_Pixmap.scaled(w,h,Qt::KeepAspectRatio));
        delete _image;
#endif   // 20221007 JYH Append
    }
    else
        ui->lbl_main_stream->clear();

#if 0   // 20221007 JYH Append
    delete _image;
#endif   // 20221007 JYH Append
}


#ifdef __DEVELOPMENT__
void MainWindow::on_spb_set_blue_valueChanged(double arg1)
{
    agentCuRAS->getHanpiece()->sendPacket(CMD_ACT_PC_TO_PROBE, SET_BLUE_OV7725, arg1*10);
}

void MainWindow::on_spb_set_red_valueChanged(double arg1)
{
    agentCuRAS->getHanpiece()->sendPacket(CMD_ACT_PC_TO_PROBE, SET_RED_OV7725, arg1*10);
}

void MainWindow::on_spb_set_green_valueChanged(double arg1)
{
    agentCuRAS->getHanpiece()->sendPacket(CMD_ACT_PC_TO_PROBE, SET_GREEN_OV7725, arg1*10);
}

void MainWindow::on_spb_set_brightness_valueChanged(int arg1)
{
    agentCuRAS->getHanpiece()->sendPacket(CMD_ACT_PC_TO_PROBE, SET_BRIGHTNESS_OV7725, arg1);
}

void MainWindow::on_spb_set_contrast_valueChanged(int arg1)
{
    agentCuRAS->getHanpiece()->sendPacket(CMD_ACT_PC_TO_PROBE, SET_CONTRAST_OV7725, arg1);
}

void MainWindow::on_spb_set_usat_valueChanged(int arg1)
{
    agentCuRAS->getHanpiece()->sendPacket(CMD_ACT_PC_TO_PROBE, SET_USAT_OV7725, arg1);
}

void MainWindow::on_spb_set_vsat_valueChanged(int arg1)
{
    agentCuRAS->getHanpiece()->sendPacket(CMD_ACT_PC_TO_PROBE, SET_VSAT_OV7725, arg1);
}

void MainWindow::on_btn_reset_clicked()
{
    agentCuRAS->getHanpiece()->sendPacket(CMD_ACT_PC_TO_PROBE, RESET_OV7725, 0);
}


void MainWindow::on_spb_set_gain_valueChanged(double arg1)
{
    agentCuRAS->getHanpiece()->sendPacket(CMD_ACT_PC_TO_PROBE, SET_GAIN_OV7725, arg1*10);
}

void MainWindow::on_spb_set_vsat_2_valueChanged(int arg1)
{
    agentCuRAS->getHanpiece()->sendPacket(CMD_ACT_PC_TO_PROBE, SET_EXPOSURE_OV7725, arg1);

}

void MainWindow::on_pushButton_clicked()
{
    bool bStatus = false;

    uint8_t registerId = ui->register_2->text().toUInt(&bStatus, 16);
    uint8_t registerVal = ui->register_value->text().toUInt(&bStatus, 16);

    qDebug() << "regis" << registerId;
    qDebug() << "value" << registerVal;

    qDebug() << "ddd" << (registerId << 8 | registerVal);


    agentCuRAS->getHanpiece()->sendPacket(CMD_ACT_PC_TO_PROBE, SET_CAMERA_REGISTER, (int)(registerId << 8 | registerVal));
}
#endif // __DEVELOPMENT__

