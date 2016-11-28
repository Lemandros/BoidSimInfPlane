#ifndef PLOTCLASS_H
#define PLOTCLASS_H
#include <QMainWindow>
#include "qcustomplot.h"
#include <QWidget>
#include "boidsim2d.h"

namespace Ui {
class PlotClass;
}

class PlotClass : public QWidget
{
    Q_OBJECT
public:

    BoidSim2D* sim;
    QString defaultPath;
    explicit PlotClass(QWidget *parent = 0, QString defaultPath = "", BoidSim2D* sim = 0);
    ~PlotClass();
private:
    Ui::PlotClass *ui;
private slots:
    void on_printPushButton_clicked( );
    void on_plotPlotAllPushButton_clicked( );
    void on_plotLogxCheckBox_toggled(bool checked);
    void on_plotLogyCheckBox_toggled(bool checked);
    void PlotSetLog(bool axis, bool checked, QCustomPlot* plot);
    void on_plotPushButton_clicked( );
    void on_plotMinSpinBox_valueChanged(int arg1);
    void on_plotMaxSpinBox_valueChanged(int arg1);
    void on_plotMaxPushButton_clicked( );
    void PrintFunction(QComboBox* comboBox, QSpinBox* minSpin, QSpinBox* maxSpin, QString fileName);
    void PlotAll(QCustomPlot* plot, QComboBox* comboBox, QSpinBox* minSpin, QSpinBox* maxSpin);
    void PlotFunction(QCustomPlot* plot, QComboBox* comboBox, QSpinBox* minSpin, QSpinBox* maxSpin);


};

#endif // PLOTCLASS_H
