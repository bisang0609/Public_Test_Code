#ifndef DEBUGWINDOW_H
#define DEBUGWINDOW_H

#include <QDialog>
#include <QLabel>
#include <QVBoxLayout>
#include <opencv2/opencv.hpp>

class DebugWindow : public QDialog {
    Q_OBJECT
public:
    explicit DebugWindow(QWidget *parent = nullptr) : QDialog(parent) {
        setWindowTitle("실시간 분석 모니터");
        label = new QLabel(this);
        QVBoxLayout *layout = new QVBoxLayout(this);
        layout->addWidget(label);
        resize(900, 450); // 가로로 원본(450) + bin(450) 배치용
    }

public slots:
    void updateImages(const cv::Mat& original, const cv::Mat& bin) {
        if (original.empty() || bin.empty()) return;

        // 1. 각각 리사이즈 (450x450)
        cv::Mat resOrig, resBin, combined;
        cv::resize(original, resOrig, cv::Size(450, 450));
        cv::resize(bin, resBin, cv::Size(450, 450));

        // 2. bin 영상 컬러 변환 (1채널 -> 3채널)
        if (resBin.channels() == 1)
            cv::cvtColor(resBin, resBin, cv::COLOR_GRAY2BGR);

        // 3. 가로로 합치기 (900x450)
        cv::hconcat(resOrig, resBin, combined);

        // 4. QImage로 변환하여 출력
        QImage qimg(combined.data, combined.cols, combined.rows, combined.step, QImage::Format_BGR888);
        label->setPixmap(QPixmap::fromImage(qimg));
    }

private:
    QLabel *label;
};

#endif
