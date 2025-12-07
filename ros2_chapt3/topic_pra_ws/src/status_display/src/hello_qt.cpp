#include <QApplication>
#include <QLabel>
#include <QString>


int main(int argc, char** argv) {

    QApplication a(argc,argv);

    QLabel* label = new QLabel();
    
    QString message = QString::fromStdString("Hello Qt6");

    label->setText(message);
    label->show();
    a.exec();


    return 0;
}