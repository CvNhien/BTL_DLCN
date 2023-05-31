#include "widget.h"
#include "ui_widget.h"

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);

    // Disable maximizing
    setFixedSize(width(), height());

    // Adding title for widget
    QWidget::setWindowTitle("Serial Port Example");

    // Ports
    QList<QSerialPortInfo> ports = info.availablePorts();
    QList<QString> stringPorts;
    for(int i = 0 ; i < ports.size() ; i++){
        stringPorts.append(ports.at(i).portName());
    }
    ui->comboBox->addItems(stringPorts);

    // Baud Rate Ratios
    QList<qint32> baudRates = info.standardBaudRates(); // What baudrates does my computer support ?
    QList<QString> stringBaudRates;
    for(int i = 0 ; i < baudRates.size() ; i++){
        stringBaudRates.append(QString::number(baudRates.at(i)));
    }
    ui->comboBox_2->addItems(stringBaudRates);

    // Data Bits
    ui->comboBox_3->addItem("5");
    ui->comboBox_3->addItem("6");
    ui->comboBox_3->addItem("7");
    ui->comboBox_3->addItem("8");

    // Stop Bits
    ui->comboBox_4->addItem("1 Bit");
    ui->comboBox_4->addItem("1,5 Bits");
    ui->comboBox_4->addItem("2 Bits");

    // Parities
    ui->comboBox_5->addItem("No Parity");
    ui->comboBox_5->addItem("Even Parity");
    ui->comboBox_5->addItem("Odd Parity");
    ui->comboBox_5->addItem("Mark Parity");
    ui->comboBox_5->addItem("Space Parity");

    //Flow Controls
    ui->comboBox_6->addItem("No Flow Control");
    ui->comboBox_6->addItem("Hardware Flow Control");
    ui->comboBox_6->addItem("Software Flow Control");

    ui->lineEdit_4->setEnabled(false);
    ui->lineEdit_5->setEnabled(false);
    ui->lineEdit_2->setEnabled(false);
    ui->checkBox_3->setEnabled(false);
    ui->checkBox_4->setEnabled(false);

    QFont font("Arial", 20, QFont::Bold);
    ui->lineEdit_3->setFont(font);

}

Widget::~Widget()
{
    delete ui;
}


void Widget::on_pushButton_2_clicked()
{

    QString portName = ui->comboBox->currentText();
    serialPort.setPortName(portName);

    serialPort.open(QIODevice::ReadWrite);

    if(!serialPort.isOpen()){
        ui->textBrowser->setTextColor(Qt::red);
        ui->textBrowser->append("!!!! Something went Wrong !!!!");
    }
    else {

        QString stringbaudRate = ui->comboBox_2->currentText();
        int intbaudRate = stringbaudRate.toInt();
        serialPort.setBaudRate(intbaudRate);

        QString dataBits = ui->comboBox_3->currentText();
        if(dataBits == "5 Bits") {
           serialPort.setDataBits(QSerialPort::Data5);
        }
        else if((dataBits == "6 Bits")) {
           serialPort.setDataBits(QSerialPort::Data6);
        }
        else if(dataBits == "7 Bits") {
           serialPort.setDataBits(QSerialPort::Data7);
        }
        else if(dataBits == "8 Bits"){
           serialPort.setDataBits(QSerialPort::Data8);
        }

        QString stopBits = ui->comboBox_4->currentText();
        if(stopBits == "1 Bit") {
         serialPort.setStopBits(QSerialPort::OneStop);
        }
        else if(stopBits == "1,5 Bits") {
         serialPort.setStopBits(QSerialPort::OneAndHalfStop);
        }
        else if(stopBits == "2 Bits") {
         serialPort.setStopBits(QSerialPort::TwoStop);
        }

        QString parity = ui->comboBox_5->currentText();
        if(parity == "No Parity"){
          serialPort.setParity(QSerialPort::NoParity);
        }
        else if(parity == "Even Parity"){
          serialPort.setParity(QSerialPort::EvenParity);
        }
        else if(parity == "Odd Parity"){
          serialPort.setParity(QSerialPort::OddParity);
        }
        else if(parity == "Mark Parity"){
          serialPort.setParity(QSerialPort::MarkParity);
        }
        else if(parity == "Space Parity") {
            serialPort.setParity(QSerialPort::SpaceParity);
        }


        QString flowControl = ui->comboBox_6->currentText();
        if(flowControl == "No Flow Control") {
          serialPort.setFlowControl(QSerialPort::NoFlowControl);
        }
        else if(flowControl == "Hardware Flow Control") {
          serialPort.setFlowControl(QSerialPort::HardwareControl);
        }
        else if(flowControl == "Software Flow Control") {
          serialPort.setFlowControl(QSerialPort::SoftwareControl);
        }

        //code = ui->lineEdit->text();
        code ="*";
        codeSize = code.size();
        connect(&serialPort,SIGNAL(readyRead()),this,SLOT(receiveMessage()));

    }


}

void Widget::receiveMessage()
{

    QByteArray dataBA = serialPort.readAll();
    QString data(dataBA);
    buffer.append(data);
    int index = buffer.indexOf(code);
    if(index != -1){
       QString message = buffer.mid(0,index);
       ui->textBrowser->setTextColor(Qt::blue); // Receieved message's color is blue.
       QString str3 = message;
       QString ch = ".";
       if(message.size()==5)
       {
           message.insert(3, ch);
       }
       else if(message.size()==4)
       {
       message.insert(2, ch);
       }
       ui->textBrowser->append(message);
       ui->lineEdit_3->setText(message);
       buffer.remove(0,index+codeSize);


       QString str1 = ui->lineEdit_4->text();
       QString str2 = ui->lineEdit_5->text();
       if (str3 >= str1 && ui->checkBox->isChecked()==1)
       {   // BẬT lED trước khi Close
           QString Alarm_High = "01A";
           serialPort.write(Alarm_High.toUtf8());
           //QMessageBox::warning(this, "ALARM", "NHIET DO QUA CAO");
           //serialPort.close();
           ui->checkBox_3->setChecked(true);
           //QString message_1 = "000";
           //serialPort.write(message.toUtf8());
           //ui->lineEdit_3->clear();
           //ui->checkBox_3->setChecked(true);
       }
       else if(str3 <= str2 && ui->checkBox->isChecked()==1)
       {
           QString Alarm_Low = "10A";
           serialPort.write(Alarm_Low.toUtf8());
            //QMessageBox::warning(this, "ALARM", "NHIET DO QUA THAP");
            //serialPort.close();
            ui->checkBox_4->setChecked(true);
            //QString message_2 = "000";
            //serialPort.write(message.toUtf8());
            //ui->checkBox_4->setChecked(true);
            //ui->lineEdit_3->clear();
       }
    }
}

void Widget::on_pushButton_clicked()
{
    if(ui->checkBox_2->isChecked()== 1)
    {
    QString message = ui->lineEdit_2->text();
    ui->textBrowser->setTextColor(Qt::darkGreen); // Color of message to send is green.
    ui->textBrowser->append(message);
    message = message +"*";
    serialPort.write(message.toUtf8());
    }
    else if(ui->radioButton->isChecked()||ui->radioButton_2->isChecked()||
       ui->radioButton_3->isChecked()||ui->radioButton_4->isChecked())
    {
        if(ui->radioButton->isChecked()==1)
        {
          QString message = "10*";
          serialPort.write(message.toUtf8());
        }
        else if(ui->radioButton_2->isChecked()==1)
        {
          QString message = "20*";
          serialPort.write(message.toUtf8());
        }
        else if(ui->radioButton_3->isChecked()==1)
        {
          QString message = "50*";
          serialPort.write(message.toUtf8());
        }
        else if(ui->radioButton_4->isChecked()==1)
        {
          QString message = "99*";
          serialPort.write(message.toUtf8());
        }
    }
    else
    {
        QString message = "10*";
        serialPort.write(message.toUtf8());
    }
}

// Button of Disconnect
void Widget::on_pushButton_3_clicked()
{
    serialPort.close();
}

// Button of Refresh Ports
void Widget::on_pushButton_4_clicked()
{
    ui->comboBox->clear();
    QList<QSerialPortInfo> ports = info.availablePorts();
    QList<QString> stringPorts;
    for(int i = 0 ; i < ports.size() ; i++){
        stringPorts.append(ports.at(i).portName());
    }
    ui->comboBox->addItems(stringPorts);
}

// Button of Clear
void Widget::on_pushButton_5_clicked()
{
    ui->textBrowser->clear();
}
// Button of Stop
void Widget::on_pushButton_6_clicked()
{
    QString message = "000";
    serialPort.write(message.toUtf8());
    ui->lineEdit_3->clear();
    ui->checkBox_3->setChecked(false);
    ui->checkBox_4->setChecked(false);

}

void Widget::on_checkBox_clicked()
{
    if((ui->checkBox->isChecked())== 1)
    {
        ui->lineEdit_4->setEnabled(true);
        ui->lineEdit_5->setEnabled(true);
        ui->checkBox_3->setEnabled(true);
        ui->checkBox_4->setEnabled(true);
    }
    else
    {   ui->lineEdit_4->clear();
        ui->lineEdit_5->clear();
        ui->lineEdit_4->setEnabled(false);
        ui->lineEdit_5->setEnabled(false);
        ui->checkBox_3->setEnabled(false);
        ui->checkBox_4->setEnabled(false);
    }
}


void Widget::on_checkBox_2_clicked()
{
    if((ui->checkBox_2->isChecked())== 1)
    {
        ui->lineEdit_2->setEnabled(true);
        ui->radioButton->setEnabled(false);
        ui->radioButton_2->setEnabled(false);
        ui->radioButton_3->setEnabled(false);
        ui->radioButton_4->setEnabled(false);
    }
    else
    {
        ui->lineEdit_2->setEnabled(false);
        ui->lineEdit_2->clear();
        ui->radioButton->setEnabled(true);
        ui->radioButton_2->setEnabled(true);
        ui->radioButton_3->setEnabled(true);
        ui->radioButton_4->setEnabled(true);
    }
}



