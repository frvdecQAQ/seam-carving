#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QDebug>
#include "SeamCarve.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_2_clicked();

    void on_resize_button_clicked();

    void on_pushButton_3_clicked();

    void on_gradient_button_clicked();

    void on_entropy_button_clicked();

    void on_HOG_button_clicked();

    void on_backward_button_clicked();

    void on_Forward_button_clicked();

    void on_pushButton_clicked();

    void on_pushButton_4_clicked();

private:
    Ui::MainWindow *ui;
    SeamCarve* sc;
    bool is_optimal;
    QImage mat2qimage(const cv::Mat &mat);
};
#endif // MAINWINDOW_H
