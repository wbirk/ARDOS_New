#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QElapsedTimer>
#include <QtConcurrent/qtconcurrentrun.h>
#include <QFile>

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
    Ui::MainWindow *ui;
//    QStepperPhidgets *StepperDrive[4]; // hold the instances for four steppers
//    QEncoderPhidgets *encoders; // hold the instances for four encoders
    QTimer *timer; // internal timer for the event queue
    QElapsedTimer *eTimer; //measures timestamps during operation
    QFuture<void> futureStepperBehaviourDr[4]; // holds events from concurrent threads
//    QAnalogPhidgets *ADInterface; // phidget interface for digital I/O
    bool endSwitchesDisabled; // flag that allows to turn off end switch evaluation in the mainwindow
    bool endSwitchHit[4]; // array holding end switch states
    void changeAccDrive(double, short); // change the acceleration of a drive
    void changeSpeedDrive(double, short); // change the speed of a drive
    void changeCurrentDrive(double, short); // change the current of a drive
    void stopDrive(short); // stop a drive
    void enableGUI(bool); // turn GUI elements on and off
    void logPositionData(void);
    double loopTrajectory[4][8][2]; // matrix holding steps and speeds for 4 trajectories with 8 segments, each cell contains steps and speed
    bool ardosIsInLoopMode;
    bool ardosInTriggerMode;
    bool loopWasTriggered;
    bool logFileIsOpen = false;
    QFile *trajectoryFile;
    QFile *logFile;
    QString *logFileName;

private slots:
    void localEventQueue(void); // the event queue handling user input
    void exitARDOS(void); // what follows are the slots for the GUI elements
    void changeCurrentDrive1(void);
    void stopDrive1(void);
    void changeCurrentDrive2(void);
    void stopDrive2(void);
    void changeCurrentDrive3(void);
    void stopDrive3(void);
    void changeCurrentDrive4(void);
    void stopDrive4(void);
    void goToStartPosition(void);
    void changeEndSwitchState(void);
    void emergencyStop(void);
    void doComplexMotion(void);
    void conveyTrajectoryToDrive1(void);
    void conveyTrajectoryToDrive2(void);
    void conveyTrajectoryToDrive3(void);
    void conveyTrajectoryToDrive4(void);
    void setTrajectory1Enabled(void);
    void setTrajectory1Disabled(void);
    void setGearRatio1(void);
    void setTrajectory2Enabled(void);
    void setTrajectory2Disabled(void);
    void setGearRatio2(void);
    void setTrajectory3Enabled(void);
    void setTrajectory3Disabled(void);
    void setGearRatio3(void);
    void setTrajectory4Enabled(void);
    void setTrajectory4Disabled(void);
    void setGearRatio4(void);
    void setSystemInTTLTriggerMode(void);
    void loadTrajectoryFile(void);
    void saveTrajectoryFile(void);
    void confirmLogFileName(void);

signals:
    void receivedTrigger(void);
};

#endif // MAINWINDOW_H
