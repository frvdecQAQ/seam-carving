#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    sc = new SeamCarve(1024, 1024);
    setFixedSize(1000, 700);
    is_optimal = false;
}

MainWindow::~MainWindow()
{
    delete ui;
    delete sc;
}


QImage MainWindow::mat2qimage(const cv::Mat &mat){
    if (mat.type() == CV_8UC1 || mat.type() == CV_8U){
        const uchar *pSrc = (const uchar*)mat.data;
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
        return image;
    }
    else if (mat.type() == CV_8UC3){
        const uchar *pSrc = (const uchar*)mat.data;
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    }
}

void MainWindow::on_pushButton_2_clicked(){
    QString file_path = QFileDialog::getOpenFileName(this);
    QByteArray tmp = file_path.toLatin1();
    sc->set_img_in(tmp.data());
    QImage img_in = mat2qimage(sc->img_in);
    QGraphicsScene *scene = new QGraphicsScene;
    scene->addPixmap(QPixmap::fromImage(img_in));
    ui->graphicsView->setScene(scene);
    ui->graphicsView->show();
    QString img_size = QString::number(sc->img_in.rows)+"x"+QString::number(sc->img_in.cols);
    ui->in_size_label->setText(img_size);
}

void MainWindow::on_resize_button_clicked(){
    int goal_h = ui->height->toPlainText().toInt();
    int goal_w = ui->width->toPlainText().toInt();
    sc->run(goal_h-sc->img_in.rows, goal_w-sc->img_in.cols);
    QImage img_out = mat2qimage(sc->img_out);
    QGraphicsScene *scene = new QGraphicsScene;
    scene->addPixmap(QPixmap::fromImage(img_out));
    ui->img_out_view->setScene(scene);
    ui->img_out_view->show();
    QString img_size = QString::number(sc->img_out.rows)+"x"+QString::number(sc->img_out.cols);
    ui->out_size_label->setText(img_size);
}

void MainWindow::on_pushButton_3_clicked(){
    QString file_path = QFileDialog::getSaveFileName(this);
    QByteArray tmp = file_path.toLatin1();
    sc->store_img_out(tmp.data());
}

void MainWindow::on_gradient_button_clicked(){
    sc->set_energy_choice(kGrad);
    ui->energy_function->setText("Gradient");
}

void MainWindow::on_entropy_button_clicked(){
    sc->set_energy_choice(kEntropy);
    ui->energy_function->setText("Entropy");
}

void MainWindow::on_HOG_button_clicked(){
    sc->set_energy_choice(kHOG);
    ui->energy_function->setText("HOG");
}

void MainWindow::on_backward_button_clicked(){
    sc->set_forward(false);
    ui->is_forward->setText("Backward");
}

void MainWindow::on_Forward_button_clicked(){
    sc->set_forward(true);
    ui->is_forward->setText("Forward");
}

void MainWindow::on_pushButton_clicked(){
    QString file_path = QFileDialog::getOpenFileName(this);
    QByteArray tmp = file_path.toLatin1();
    sc->set_remove_mask(tmp.data());
    sc->remove_object();
    QImage img_out = mat2qimage(sc->img_out);
    QGraphicsScene *scene = new QGraphicsScene;
    scene->addPixmap(QPixmap::fromImage(img_out));
    ui->img_out_view->setScene(scene);
    ui->img_out_view->show();
    QString img_size = QString::number(sc->img_out.rows)+"x"+QString::number(sc->img_out.cols);
    ui->out_size_label->setText(img_size);
}

void MainWindow::on_pushButton_4_clicked(){
    if(is_optimal){
        sc->set_optimal_order(false);
        is_optimal = false;
        ui->order_label->setText("normal order");
    }else{
        sc->set_optimal_order(true);
        is_optimal = true;
        ui->order_label->setText("optimal order");
    }
}
