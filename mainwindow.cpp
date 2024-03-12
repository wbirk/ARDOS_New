// basic program for the ARDOS phantom. w. birkfellner, cmpbme, mu vienna, 2017
// drive 1 has a 1:4.25 gearbox, drive 4 has a 1:5.2 gearbox
// VREALVNC.COM

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <unistd.h>
#include <qtimer.h>
#include <QElapsedTimer>
#include <QFileDialog>

//-------------------------------------------------------------------------------------
MainWindow::MainWindow(QWidget *parent):QMainWindow(parent),ui(new Ui::MainWindow) {
//    int adSerNo;
    short driveCounter;

    ui->setupUi(this);
    QTimer *timer = new QTimer(this);
    timer->start(20);  // check the main event queue every 20 ms
//    ADInterface = new QAnalogPhidgets();
//    adSerNo = ADInterface->retrievePhidgetControllerData(1);
//    ui->lcdADID->display(adSerNo); // initialize the AD interface for digital endswitches
    this->endSwitchesDisabled=false; // true if end switches are to be ignored
    this->endSwitchHit[0] = this->endSwitchHit[1] = this->endSwitchHit[2] = this->endSwitchHit[3]= 0; // it is assumed that endswitches are not hit
    this->ardosIsInLoopMode=false;
    this->ardosInTriggerMode=false;
    this->loopWasTriggered = false;
    this->logFileName = new QString(ui->leLogFileName->text());

    connect(timer, SIGNAL(timeout()), this, SLOT(localEventQueue())); // trigger check of the event queue when timer is > 50 ms
    connect(ui->pbExit, SIGNAL(clicked()), this, SLOT(exitARDOS())); // exit the program
    connect(ui->sbCurrent2,SIGNAL(valueChanged(double)), this, SLOT(changeCurrentDrive2())); // same as above for the other drives
    connect(ui->sbCurrent2,SIGNAL(valueChanged(double)), this, SLOT(changeCurrentDrive2()));
    connect(ui->sbCurrent3,SIGNAL(valueChanged(double)), this, SLOT(changeCurrentDrive3()));
    connect(ui->sbCurrent4,SIGNAL(valueChanged(double)), this, SLOT(changeCurrentDrive4()));
    connect(ui->pbInitArdos,SIGNAL(clicked()), this, SLOT(goToStartPosition())); // go to starting position according to endswitches
    connect(ui->cbDisableES,SIGNAL(released()),this, SLOT(changeEndSwitchState())); // for positioning, end switches can be disabled here
    connect(ui->pbEmergencyStop, SIGNAL(clicked()), this, SLOT(emergencyStop())); // kill all drives
    connect(ui->pbStartARDOS,SIGNAL(clicked()),this,SLOT(doComplexMotion())); // operate all 4 drives with given parameters
    connect(ui->pbSendTraj1, SIGNAL(clicked()),this,SLOT(conveyTrajectoryToDrive1(void)));
    connect(ui->pbSendTraj2, SIGNAL(clicked()),this,SLOT(conveyTrajectoryToDrive2(void)));
    connect(ui->pbSendTraj3, SIGNAL(clicked()),this,SLOT(conveyTrajectoryToDrive3(void)));
    connect(ui->pbSendTraj4, SIGNAL(clicked()),this,SLOT(conveyTrajectoryToDrive4(void)));
    connect(ui->pbEnableDrive1, SIGNAL(clicked()), this, SLOT(setTrajectory1Enabled(void)));
    connect(ui->pbDisableDrive1, SIGNAL(clicked()), this, SLOT(setTrajectory1Disabled(void)));
    connect(ui->pbEnableDrive2, SIGNAL(clicked()), this, SLOT(setTrajectory2Enabled(void)));
    connect(ui->pbDisableDrive2, SIGNAL(clicked()), this, SLOT(setTrajectory2Disabled(void)));
    connect(ui->pbEnableDrive3, SIGNAL(clicked()), this, SLOT(setTrajectory3Enabled(void)));
    connect(ui->pbDisableDrive3, SIGNAL(clicked()), this, SLOT(setTrajectory3Disabled(void)));
    connect(ui->pbEnableDrive4, SIGNAL(clicked()), this, SLOT(setTrajectory4Enabled(void)));
    connect(ui->pbDisableDrive4, SIGNAL(clicked()), this, SLOT(setTrajectory4Disabled(void)));
    connect(ui->sbGear1, SIGNAL(valueChanged(double)), this, SLOT(setGearRatio1(void)));
    connect(ui->sbGear2, SIGNAL(valueChanged(double)), this, SLOT(setGearRatio2(void)));
    connect(ui->sbGear3, SIGNAL(valueChanged(double)), this, SLOT(setGearRatio3(void)));
    connect(ui->sbGear4, SIGNAL(valueChanged(double)), this, SLOT(setGearRatio4(void)));
    connect(ui->pbWaitForTrigger, SIGNAL(clicked()), this, SLOT(setSystemInTTLTriggerMode(void)));
    connect(ui->pbLoadTraj, SIGNAL(clicked()), this, SLOT(loadTrajectoryFile()));
    connect(ui->pbSaveTraj, SIGNAL(clicked()), this, SLOT(saveTrajectoryFile()));
    connect(ui->pbLFileNameConfirm, SIGNAL(clicked()), this, SLOT(confirmLogFileName()));
    connect(this, SIGNAL(receivedTrigger()), this, SLOT(doComplexMotion()), Qt::QueuedConnection);

    for (driveCounter = 0; driveCounter < 4; driveCounter++) {
//        StepperDrive[driveCounter] = new QStepperPhidgets();
    } // initialize the 4 stepper controllers for drives 1-4

//    StepperDrive[0]->setKinetics((ui->sbCurrent1->value()),3);
//    setGearRatio1();
//    StepperDrive[1]->setKinetics((ui->sbCurrent2->value()),3);
//    setGearRatio2();
//    StepperDrive[2]->setKinetics((ui->sbCurrent3->value()),3);
//    setGearRatio3();
//    StepperDrive[3]->setKinetics((ui->sbCurrent4->value()),3); // set the coil current for each controller according to input
//    setGearRatio4();
 //   this->encoders = new QEncoderPhidgets();
    this->eTimer=new QElapsedTimer();
}

//-------------------------------------------------------------------------------------
MainWindow::~MainWindow() {
//    delete StepperDrive[0];
//    delete StepperDrive[1];
//    delete StepperDrive[2];
//    delete StepperDrive[3];
//    delete encoders;
    delete eTimer;
//    delete ADInterface; // destructor - shuts down all phidget interfaces
    delete ui;
}

//-------------------------------------------------------------------------------------
void MainWindow::localEventQueue(void) {
    short digitalInputs[8],inCount;
    double stepsFromEncoder[4];

    for (inCount=0; inCount < 8; inCount++) { // check all digital inputs for end switches
//        digitalInputs[inCount]=ADInterface->getDigitalInput(inCount);
    }
//    ui->cbT1->setChecked(StepperDrive[0]->getLoopFlag());
//    ui->cbT2->setChecked(StepperDrive[1]->getLoopFlag());
//    ui->cbT3->setChecked(StepperDrive[2]->getLoopFlag());
//    ui->cbT4->setChecked(StepperDrive[3]->getLoopFlag());
    if (this->endSwitchesDisabled == false) { // it is possible to disable endswitches with a checkbox for careful repositioning
        if (digitalInputs[5]==0) { // end switch for drive 1 is connected to digital input 5
            this->stopDrive1(); // if end switch is hit, stop the drive ...
            this->endSwitchHit[0] = true; // and set a flag to "true"
            ui->cbDr1->setChecked(false); // display a checkbox whether the drive is active
        } else {
            ui->cbDr1->setChecked(true);
            this->endSwitchHit[0] = false;
        }
        if (digitalInputs[3]==0) { // end switch for drive 2 is connected to digital input 3
            this->stopDrive2();
            this->endSwitchHit[1] = true;
            ui->cbDr2->setChecked(false);
        } else {
            ui->cbDr2->setChecked(true);
            this->endSwitchHit[1] = false;
        }
        if (digitalInputs[4]==0) { // end switch for drive 3 is connected to digital input 4
            this->stopDrive3();
            this->endSwitchHit[2] = true;
            ui->cbDr3->setChecked(false);
        } else {
            ui->cbDr3->setChecked(true);
           this->endSwitchHit[2] = false;
        }
        if (digitalInputs[7]==1) { // end switch for drive 4 is connected to digital input 7
            this->stopDrive4();
            this->endSwitchHit[3] = true;
            ui->cbDr4->setChecked(false);
        } else {
            ui->cbDr4->setChecked(true);
            this->endSwitchHit[3] = false;
        }
    }

    if (this->ardosIsInLoopMode == true) {
//        stepsFromEncoder[0]=this->encoders->getTopicalReadingFromEncoder(0)*0.002;
//        stepsFromEncoder[1]=this->encoders->getTopicalReadingFromEncoder(1)*0.002;
//        stepsFromEncoder[2]=this->encoders->getTopicalReadingFromEncoder(2)*0.002;
//        stepsFromEncoder[3]=this->encoders->getTopicalReadingFromEncoder(3)*(-0.057142857); // thes are the conversion factors for the encoders
        ui->lcdPos1->display(QString::number(stepsFromEncoder[0],'g',3));
        ui->lcdPos2->display(QString::number(stepsFromEncoder[1],'g',3));
        ui->lcdPos3->display(QString::number(stepsFromEncoder[2],'g',3));
        ui->lcdPos4->display(QString::number(stepsFromEncoder[3],'g',3));
        ui->lcdTimestamp->display(QString::number(this->eTimer->elapsed()/1000.0, 'g', 3));
        if ((ui->cbLogPositions->isChecked() == true)  && (this->logFileIsOpen == true)) {
            logPositionData();
        }
    }
     if (this->ardosInTriggerMode == true) {
         if (digitalInputs[6] == 0) {
             this->ardosInTriggerMode = false;
             this->loopWasTriggered = true;
             emit receivedTrigger();
         }
     }
}

//-------------------------------------------------------------------------------------
void MainWindow::changeAccDrive(double acc, short whatDrive) {
//    this->StepperDrive[whatDrive]->setKinetics(acc,1); // change acceleration; the SLOT is defined below for each button
    acc = 1;
    whatDrive = 1;
}

//-------------------------------------------------------------------------------------
void MainWindow::changeSpeedDrive(double speed, short whatDrive) {
//    this->StepperDrive[whatDrive]->setKinetics(speed,2); // change speed; the SLOT is defined below for each button
    speed = 1;
    whatDrive = 1;
}

//--------------------------------------------------------------------------------------
void MainWindow::changeCurrentDrive(double current, short whatDrive) {
//    this->StepperDrive[whatDrive]->setKinetics(current,3); // change coil current; the SLOT is defined below for each button
    current = 1;
    whatDrive = 1;
}

//--------------------------------------------------------------------------------------
void MainWindow::stopDrive(short whatDrive) { // stop a drive; the SLOT is defined below for each button
//       this->StepperDrive[whatDrive]->setDriveActive(false);
//       this->StepperDrive[whatDrive]->shutDownDrive();
    whatDrive = 1;
}

//--------------------------------------------------------------------------------------
void MainWindow::exitARDOS(void) { // kill the program
    stopDrive1();
    stopDrive2();
    stopDrive3();
    stopDrive4();
    this->logFile->close();
    exit(0);
}
//--------------------------------------------------------------------------------------
// position the drives 1-4 to endswitches

void MainWindow::goToStartPosition(void) {
    double speeds[4];
    short driveCount, digitalInputDr1,digitalInputDr2,digitalInputDr3, digitalInputDr4;
    QElapsedTimer *wait;

    ui->lcdTimestamp->display(0);
    this->endSwitchesDisabled=false;
    this->enableGUI(false); // kill all GUI elements during operation
    ui->cbDisableES->setChecked(false); // endswitches must be enabled
    speeds[0] = 5000*4.25; // drive 1 has a 1:4.25 gearbox
    speeds[1] = 5000;
    speeds[2] = 5000;
    speeds[3] = 5000*5.18; // drive 4 has a 5.18 gearbox
    for (driveCount = 0; driveCount < 4; driveCount++) {
        changeSpeedDrive(speeds[driveCount], driveCount);
//        this->StepperDrive[driveCount]->setDriveActive(true);
    }

    do {
//        this->StepperDrive[0]->sendSteps(-2250); // tell the drive to pull back for 500*4.25 microsteps
        localEventQueue();
    } while (this->endSwitchHit[0]==false); // until the endswitch is activated

    do {
//        this->StepperDrive[1]->sendSteps(-500); // tell the drive to pull back for 500 microsteps
        localEventQueue();
    } while (this->endSwitchHit[1]==false); // until the endswitch is activated

    do {
//        this->StepperDrive[2]->sendSteps(-500); // tell the drive to pull back for 500 microsteps
        localEventQueue();
    } while (this->endSwitchHit[2]==false); // until the endswitch is activated
    do {
//        this->StepperDrive[3]->sendSteps(500); // tell the drive to pull back for 500*5.18 microsteps
        localEventQueue();
    } while (this->endSwitchHit[3]==false); // until the endswitch is activated

    this->endSwitchesDisabled = true; // now, the endswitches are disabled in the event loop for moving the drives away from them
    ui->cbDisableES->setChecked(true);
    do {
//        this->StepperDrive[0]->sendSteps(4250); // drive 1 has a 1:4.25 gearbox
            QCoreApplication::processEvents(QEventLoop::AllEvents);
//            digitalInputDr1=ADInterface->getDigitalInput(5); // endswitch for drive 1 is read ...
        digitalInputDr1 = 1;//#################################################################################################################################################
        if (digitalInputDr1 == 1) {
            this->endSwitchHit[0] = false;
        }
        QCoreApplication::processEvents(QEventLoop::AllEvents);
    } while (this->endSwitchHit[0] == true); // until it is free
    do {
//        this->StepperDrive[1]->sendSteps(100);
            QCoreApplication::processEvents(QEventLoop::AllEvents);
//            digitalInputDr2=ADInterface->getDigitalInput(3);
//#################################################################################################################################################
        digitalInputDr2 = 1;
        digitalInputDr3 = 1;
        digitalInputDr4 = 1;
//#################################################################################################################################################
        if (digitalInputDr2 == 1) {
            this->endSwitchHit[1] = false;
        }
        QCoreApplication::processEvents(QEventLoop::AllEvents);
    } while (this->endSwitchHit[1] == true);
    do {
//        this->StepperDrive[2]->sendSteps(100);
            QCoreApplication::processEvents(QEventLoop::AllEvents);
//            digitalInputDr3=ADInterface->getDigitalInput(4);
        if (digitalInputDr3 == 1) {
            this->endSwitchHit[2] = false;
            QCoreApplication::processEvents(QEventLoop::AllEvents);
        }
    } while (this->endSwitchHit[2] == true);

    do {
//        this->StepperDrive[3]->sendSteps(-100);
            QCoreApplication::processEvents(QEventLoop::AllEvents);
//            digitalInputDr4=ADInterface->getDigitalInput(7);
        if (digitalInputDr4 == 0) {
            this->endSwitchHit[3] = false;
            QCoreApplication::processEvents(QEventLoop::AllEvents);
        }
    } while (this->endSwitchHit[3] == true);
    // repeating the procedure for all other drives

    this->endSwitchesDisabled = false; // now, the endswitches are enabled again

    ui->cbDisableES->setChecked(false);
    wait = new QElapsedTimer();
    wait->start();
    while (wait->elapsed() < 500) {
        QCoreApplication::processEvents(QEventLoop::AllEvents);
    }
    delete wait; // just give the GUI some time to update
    //this->StepperDrive[0]->sendSteps(226361); // drive 1 has a 1:4.25 gearbox --40861,2 (3cm)+ 185500--239982
//    this->StepperDrive[0]->sendSteps(69485);

    QCoreApplication::processEvents(QEventLoop::AllEvents);
//    this->StepperDrive[1]->sendSteps(1000);
    QCoreApplication::processEvents(QEventLoop::AllEvents);
    // this->StepperDrive[2]->sendSteps(65022); // move drives further away from endswitches --45750 + 9636 +9636 (3+3cm)--for insert film 2 /68234
//    this->StepperDrive[2]->sendSteps(19420); // changed due to redesign
//    this->StepperDrive[3]->sendSteps(-1000); // changed due to redesign
    QCoreApplication::processEvents(QEventLoop::AllEvents);
    this->enableGUI(true); // enable GUI again
    for (driveCount = 0; driveCount < 4; driveCount++) {
//        this->encoders->resetEncoder(driveCount);
    }
    ui->lcdPos1->display(0);
    ui->lcdPos2->display(0);
    ui->lcdPos3->display(0);
    ui->lcdPos4->display(0);
}

//--------------------------------------------------------------------------------------
void MainWindow::changeEndSwitchState(void) { // just a helper that keeps track of end switch states
    if (ui->cbDisableES->isChecked()==true) {
        this->endSwitchesDisabled = true;
    } else {
        this->endSwitchesDisabled = false;
    }
}

//--------------------------------------------------------------------------------------
void MainWindow::emergencyStop(void) { // stop all drives immediately

    stopDrive1();
    stopDrive2();
    stopDrive3();
    stopDrive4();
    this->ardosIsInLoopMode = false;
    this->ardosInTriggerMode = false;
    this->loopWasTriggered =false;
    if ((ui->cbLogPositions->isChecked() == true) && (this->logFileIsOpen == true)){
        this->logFile->write("-------------------------\n");
    }
}

//--------------------------------------------------------------
void MainWindow::enableGUI(bool setEnabled) { // diable or enable GUI elements during initialisation or operation

    ui->cbDisableES->setEnabled(setEnabled);
    ui->pbInitArdos->setEnabled(setEnabled);
    ui->pbStartARDOS->setEnabled(setEnabled);
    ui->ComplexTab1->setEnabled(setEnabled);
    ui->ComplexTab2->setEnabled(setEnabled);
    ui->ComplexTab3->setEnabled(setEnabled);
    ui->ComplexTab4->setEnabled(setEnabled);
    ui->MotorTab->setEnabled(setEnabled);
    ui->gbTrajectories->setEnabled(setEnabled);
}

//----------------------------------------------------------------
void MainWindow::doComplexMotion(void) { // loop all 4 drives
    short drCount;
    QElapsedTimer *shortDelay;

    if ((ui->cbT1->isChecked() == true) ||
        (ui->cbT2->isChecked() == true) ||
        (ui->cbT3->isChecked() == true) ||
        (ui->cbT4->isChecked() == true)) {
        shortDelay = new QElapsedTimer;
        this->enableGUI(false);
        this->ardosIsInLoopMode = true;
        ui->cbDisableES->setEnabled(false);
        ui->pbInitArdos->setEnabled(false);
        ui->pbStartARDOS->setEnabled(false);
        for (drCount = 0; drCount < 4; drCount++) {
//            this->StepperDrive[drCount]->setDriveActive(true);
        }
        this->eTimer->start();
        do {
// now start the periodic motion for all four drives
            for (drCount = 0; drCount < 4; drCount++) {

//                futureStepperBehaviourDr[drCount] = QtConcurrent::run(this->StepperDrive[drCount], &QStepperPhidgets::loopDriveComplex);
                shortDelay->start();
                while (shortDelay->elapsed() < 10) {}
            }
            localEventQueue();
            QCoreApplication::processEvents(QEventLoop::AllEvents);
            while (!futureStepperBehaviourDr[0].isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents);
            }
            while (!futureStepperBehaviourDr[1].isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents);
            }
            while (!futureStepperBehaviourDr[2].isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents);
            }
            while (!futureStepperBehaviourDr[3].isFinished()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents);
            }
//            StepperDrive[0]->setTrajectoryDoneFlag(false);
//            StepperDrive[1]->setTrajectoryDoneFlag(false);
//            StepperDrive[2]->setTrajectoryDoneFlag(false);
//            StepperDrive[3]->setTrajectoryDoneFlag(false);


        } while (ardosIsInLoopMode == true);
    // ok, this is complicated - each drive is started asynchronously in a separate thread - "futureStepperBehaviour keeops track whether the routine ended
    // in the respective thread. formally, the program is stuck in this routine but operates for other threads and processes signals in the event queue
        for (drCount = 0; drCount < 4; drCount++) {
//            this->StepperDrive[drCount]->setDriveActive(false);
        }
        this->enableGUI(true);
        this->endSwitchesDisabled = false;
//        this->goToStartPosition();
        ui->pbWaitForTrigger->setEnabled(true);
        delete shortDelay;
    }
}

//--------------------------------------------------------------------------------------
void MainWindow::setSystemInTTLTriggerMode(void) {
    if ((ui->cbT1->isChecked() == true) ||
        (ui->cbT2->isChecked() == true) ||
        (ui->cbT3->isChecked() == true) ||
        (ui->cbT4->isChecked() == true)) {
            this->ardosInTriggerMode = true;
            this->enableGUI(false);
            ui->pbWaitForTrigger->setEnabled(false);
    }
}

//--------------------------------------------------------------------------------------
//----------------------------------------- drive specific slots - one slot for each ---
//----------------------------------------- widget -------------------------------------
//--------------------------------------------------------------------------------------
void MainWindow::stopDrive1(void) {
    this->stopDrive(0);
}

//--------------------------------------------------------------------------------------
void MainWindow::changeCurrentDrive1(void) {
    double curr;

    curr=ui->sbCurrent1->value();
    this->changeCurrentDrive(curr, 0);
}
//--------------------------------------------------------------------------------------

void MainWindow::stopDrive2(void) {
    this->stopDrive(1);
}

//--------------------------------------------------------------------------------------
void MainWindow::changeCurrentDrive2(void) {
    double curr;

    curr=ui->sbCurrent2->value();
    this->changeCurrentDrive(curr,1);
}
//--------------------------------------------------------------------------------------
void MainWindow::stopDrive3(void) {
    this->stopDrive(2);
}

//--------------------------------------------------------------------------------------
void MainWindow::changeCurrentDrive3(void) {
    double curr;

    curr=ui->sbCurrent3->value();
    this->changeCurrentDrive(curr,2);
}
//--------------------------------------------------------------------------------------

void MainWindow::stopDrive4(void) {
    this->stopDrive(3);
}

//--------------------------------------------------------------------------------------
void MainWindow::changeCurrentDrive4(void) {
    double curr;

    curr=ui->sbCurrent4->value();
    this->changeCurrentDrive(curr, 3);
}

//--------------------------------------------------------------------------------------
void MainWindow::setTrajectory1Enabled(void) {
//    StepperDrive[0]->setLoopFlag(true);
    ui->pbDisableDrive1->setEnabled(true);
    ui->pbEnableDrive1->setEnabled(false);
}

//--------------------------------------------------------------------------------------
void MainWindow::setTrajectory1Disabled(void) {
//    StepperDrive[0]->setLoopFlag(false);
    ui->pbDisableDrive1->setEnabled(false);
    ui->pbEnableDrive1->setEnabled(true);
}

//--------------------------------------------------------------------------------------
void MainWindow::setGearRatio1(void) {
//    StepperDrive[0]->setGearRatio(ui->sbGear1->value());
}
//--------------------------------------------------------------------------------------
void MainWindow::setTrajectory2Enabled(void) {
//    StepperDrive[1]->setLoopFlag(true);
    ui->pbDisableDrive2->setEnabled(true);
    ui->pbEnableDrive2->setEnabled(false);
}

//--------------------------------------------------------------------------------------
void MainWindow::setTrajectory2Disabled(void) {
//    StepperDrive[1]->setLoopFlag(false);
    ui->pbDisableDrive2->setEnabled(false);
    ui->pbEnableDrive2->setEnabled(true);
}

//--------------------------------------------------------------------------------------
void MainWindow::setGearRatio2(void) {
//    StepperDrive[1]->setGearRatio(ui->sbGear2->value());
}
//--------------------------------------------------------------------------------------
void MainWindow::setTrajectory3Enabled(void) {
//    StepperDrive[2]->setLoopFlag(true);
    ui->pbDisableDrive3->setEnabled(true);
    ui->pbEnableDrive3->setEnabled(false);
}

//--------------------------------------------------------------------------------------
void MainWindow::setTrajectory3Disabled(void) {
//    StepperDrive[2]->setLoopFlag(false);
    ui->pbDisableDrive3->setEnabled(false);
    ui->pbEnableDrive3->setEnabled(true);
}

//--------------------------------------------------------------------------------------
void MainWindow::setGearRatio3(void) {
//    StepperDrive[2]->setGearRatio(ui->sbGear3->value());

}
//--------------------------------------------------------------------------------------
void MainWindow::setTrajectory4Enabled(void) {
//    StepperDrive[3]->setLoopFlag(true);
    ui->pbDisableDrive4->setEnabled(true);
    ui->pbEnableDrive4->setEnabled(false);
}

//--------------------------------------------------------------------------------------
void MainWindow::setTrajectory4Disabled(void) {
//    StepperDrive[3]->setLoopFlag(false);
    ui->pbDisableDrive4->setEnabled(false);
    ui->pbEnableDrive4->setEnabled(true);
}

//--------------------------------------------------------------------------------------
void MainWindow::setGearRatio4(void) {
//    StepperDrive[3]->setGearRatio(ui->sbGear4->value());
}

//-------------------------------------------------------------------------------------
// store the trajectory from the GUI to drive 1
void MainWindow::conveyTrajectoryToDrive1(void) {
/*    StepperDrive[0]->setLoop(0,0,ui->sbSteps1_1->value());
    StepperDrive[0]->setLoop(0,1,ui->sbSteps1_2->value());
    StepperDrive[0]->setLoop(0,2,ui->sbSteps1_3->value());
    StepperDrive[0]->setLoop(0,3,ui->sbSteps1_4->value());
    StepperDrive[0]->setLoop(0,4,ui->sbSteps1_5->value());
    StepperDrive[0]->setLoop(0,5,ui->sbSteps1_6->value());
    StepperDrive[0]->setLoop(0,6,ui->sbSteps1_7->value());
    StepperDrive[0]->setLoop(0,7,ui->sbSteps1_8->value());
    StepperDrive[0]->setLoop(1,0,ui->sbSpeed1_1->value());
    StepperDrive[0]->setLoop(1,1,ui->sbSpeed1_2->value());
    StepperDrive[0]->setLoop(1,2,ui->sbSpeed1_3->value());
    StepperDrive[0]->setLoop(1,3,ui->sbSpeed1_4->value());
    StepperDrive[0]->setLoop(1,4,ui->sbSpeed1_5->value());
    StepperDrive[0]->setLoop(1,5,ui->sbSpeed1_6->value());
    StepperDrive[0]->setLoop(1,6,ui->sbSpeed1_7->value());
    StepperDrive[0]->setLoop(1,7,ui->sbSpeed1_8->value());
    StepperDrive[0]->setLoop(2,0,ui->sbCRest1_1->value());
    StepperDrive[0]->setLoop(2,1,ui->sbCRest1_2->value());
    StepperDrive[0]->setLoop(2,2,ui->sbCRest1_3->value());
    StepperDrive[0]->setLoop(2,3,ui->sbCRest1_4->value());
    StepperDrive[0]->setLoop(2,4,ui->sbCRest1_5->value());
    StepperDrive[0]->setLoop(2,5,ui->sbCRest1_6->value());
    StepperDrive[0]->setLoop(2,6,ui->sbCRest1_7->value());
    StepperDrive[0]->setLoop(2,7,ui->sbCRest1_8->value()); */
    ui->pbDisableDrive1->setEnabled(true);
    ui->pbEnableDrive1->setEnabled(false);
}

//-------------------------------------------------------------------------------------
// store the trajectory from the GUI to drive 2
void MainWindow::conveyTrajectoryToDrive2(void) {
/*    StepperDrive[1]->setLoop(0,0,ui->sbSteps2_1->value());
    StepperDrive[1]->setLoop(0,1,ui->sbSteps2_2->value());
    StepperDrive[1]->setLoop(0,2,ui->sbSteps2_3->value());
    StepperDrive[1]->setLoop(0,3,ui->sbSteps2_4->value());
    StepperDrive[1]->setLoop(0,4,ui->sbSteps2_5->value());
    StepperDrive[1]->setLoop(0,5,ui->sbSteps2_6->value());
    StepperDrive[1]->setLoop(0,6,ui->sbSteps2_7->value());
    StepperDrive[1]->setLoop(0,7,ui->sbSteps2_8->value());
    StepperDrive[1]->setLoop(1,0,ui->sbSpeed2_1->value());
    StepperDrive[1]->setLoop(1,1,ui->sbSpeed2_2->value());
    StepperDrive[1]->setLoop(1,2,ui->sbSpeed2_3->value());
    StepperDrive[1]->setLoop(1,3,ui->sbSpeed2_4->value());
    StepperDrive[1]->setLoop(1,4,ui->sbSpeed2_5->value());
    StepperDrive[1]->setLoop(1,5,ui->sbSpeed2_6->value());
    StepperDrive[1]->setLoop(1,6,ui->sbSpeed2_7->value());
    StepperDrive[1]->setLoop(1,7,ui->sbSpeed2_8->value());
    StepperDrive[1]->setLoop(2,0,ui->sbCRest2_1->value());
    StepperDrive[1]->setLoop(2,1,ui->sbCRest2_2->value());
    StepperDrive[1]->setLoop(2,2,ui->sbCRest2_3->value());
    StepperDrive[1]->setLoop(2,3,ui->sbCRest2_4->value());
    StepperDrive[1]->setLoop(2,4,ui->sbCRest2_5->value());
    StepperDrive[1]->setLoop(2,5,ui->sbCRest2_6->value());
    StepperDrive[1]->setLoop(2,6,ui->sbCRest2_7->value());
    StepperDrive[1]->setLoop(2,7,ui->sbCRest2_8->value()); */
    ui->pbDisableDrive2->setEnabled(true);
    ui->pbEnableDrive2->setEnabled(false);
}

//-------------------------------------------------------------------------------------
// store the trajectory from the GUI to drive 3
void MainWindow::conveyTrajectoryToDrive3(void) {
/*    StepperDrive[2]->setLoop(0,0,ui->sbSteps3_1->value());
    StepperDrive[2]->setLoop(0,1,ui->sbSteps3_2->value());
    StepperDrive[2]->setLoop(0,2,ui->sbSteps3_3->value());
    StepperDrive[2]->setLoop(0,3,ui->sbSteps3_4->value());
    StepperDrive[2]->setLoop(0,4,ui->sbSteps3_5->value());
    StepperDrive[2]->setLoop(0,5,ui->sbSteps3_6->value());
    StepperDrive[2]->setLoop(0,6,ui->sbSteps3_7->value());
    StepperDrive[2]->setLoop(0,7,ui->sbSteps3_8->value());
    StepperDrive[2]->setLoop(1,0,ui->sbSpeed3_1->value());
    StepperDrive[2]->setLoop(1,1,ui->sbSpeed3_2->value());
    StepperDrive[2]->setLoop(1,2,ui->sbSpeed3_3->value());
    StepperDrive[2]->setLoop(1,3,ui->sbSpeed3_4->value());
    StepperDrive[2]->setLoop(1,4,ui->sbSpeed3_5->value());
    StepperDrive[2]->setLoop(1,5,ui->sbSpeed3_6->value());
    StepperDrive[2]->setLoop(1,6,ui->sbSpeed3_7->value());
    StepperDrive[2]->setLoop(1,7,ui->sbSpeed3_8->value());
    StepperDrive[2]->setLoop(2,0,ui->sbCRest3_1->value());
    StepperDrive[2]->setLoop(2,1,ui->sbCRest3_2->value());
    StepperDrive[2]->setLoop(2,2,ui->sbCRest3_3->value());
    StepperDrive[2]->setLoop(2,3,ui->sbCRest3_4->value());
    StepperDrive[2]->setLoop(2,4,ui->sbCRest3_5->value());
    StepperDrive[2]->setLoop(2,5,ui->sbCRest3_6->value());
    StepperDrive[2]->setLoop(2,6,ui->sbCRest3_7->value());
    StepperDrive[2]->setLoop(2,7,ui->sbCRest3_8->value());*/
    ui->pbDisableDrive3->setEnabled(true);
    ui->pbEnableDrive3->setEnabled(false);
}

//-------------------------------------------------------------------------------------
// store the trajectory from the GUI to drive 4
void MainWindow::conveyTrajectoryToDrive4(void) {
/*    StepperDrive[3]->setLoop(0,0,-ui->sbSteps4_1->value());
    StepperDrive[3]->setLoop(0,1,-ui->sbSteps4_2->value());
    StepperDrive[3]->setLoop(0,2,-ui->sbSteps4_3->value());
    StepperDrive[3]->setLoop(0,3,-ui->sbSteps4_4->value());
    StepperDrive[3]->setLoop(0,4,-ui->sbSteps4_5->value());
    StepperDrive[3]->setLoop(0,5,-ui->sbSteps4_6->value());
    StepperDrive[3]->setLoop(0,6,-ui->sbSteps4_7->value());
    StepperDrive[3]->setLoop(0,7,-ui->sbSteps4_8->value());
    StepperDrive[3]->setLoop(1,0,ui->sbSpeed4_1->value());
    StepperDrive[3]->setLoop(1,1,ui->sbSpeed4_2->value());
    StepperDrive[3]->setLoop(1,2,ui->sbSpeed4_3->value());
    StepperDrive[3]->setLoop(1,3,ui->sbSpeed4_4->value());
    StepperDrive[3]->setLoop(1,4,ui->sbSpeed4_5->value());
    StepperDrive[3]->setLoop(1,5,ui->sbSpeed4_6->value());
    StepperDrive[3]->setLoop(1,6,ui->sbSpeed4_7->value());
    StepperDrive[3]->setLoop(1,7,ui->sbSpeed4_8->value());
    StepperDrive[3]->setLoop(2,0,ui->sbCRest4_1->value());
    StepperDrive[3]->setLoop(2,1,ui->sbCRest4_2->value());
    StepperDrive[3]->setLoop(2,2,ui->sbCRest4_3->value());
    StepperDrive[3]->setLoop(2,3,ui->sbCRest4_4->value());
    StepperDrive[3]->setLoop(2,4,ui->sbCRest4_5->value());
    StepperDrive[3]->setLoop(2,5,ui->sbCRest4_6->value());
    StepperDrive[3]->setLoop(2,6,ui->sbCRest4_7->value());
    StepperDrive[3]->setLoop(2,7,ui->sbCRest4_8->value());*/
    ui->pbDisableDrive4->setEnabled(true);
    ui->pbEnableDrive4->setEnabled(false);
}

//--------------------------------------------------------------------------------------
void MainWindow::confirmLogFileName(void) {
    this->logFileName->clear();
    this->logFileName->append(ui->leLogFileName->text());
    this->logFile = new QFile();
    this->logFile->setFileName(this->logFileName->toLatin1());
    this->logFile->open(QIODevice::Append);
    this->logFile->write("TimeStamp [s]\tDrive 1 [cm]\tDrive 2 [cm]\tDrive3 [cm]\tDrive 4 [°]\n");
    ui->pbLFileNameConfirm->setEnabled(false);
    ui->leLogFileName->setEnabled(false);
    this->logFileIsOpen = true;
    ui->cbLogFileIsOpen->setChecked(true);
}


//--------------------------------------------------------------------------------------
// log positions with timestamp
void MainWindow::logPositionData(void) {
    QString *tStamp, *pos1, *pos2, *pos3, *pos4, *line;

    tStamp = new QString(QString::number(ui->lcdTimestamp->value(),'g',8));
    pos1 = new QString(QString::number(ui->lcdPos1->value(),'g',5));
    pos2 = new QString(QString::number(ui->lcdPos2->value(),'g',5));
    pos3 = new QString(QString::number(ui->lcdPos3->value(),'g',5));
    pos4 = new QString(QString::number(ui->lcdPos4->value(),'g',5));
    line=new QString();
    line->append(tStamp);
    line->append("\t");
    line->append(pos1);
    line->append("\t");
    line->append(pos2);
    line->append("\t");
    line->append(pos3);
    line->append("\t");
    line->append(pos4);
    line->append("\n");
    this->logFile->write(line->toLatin1());
    delete line;
}
//--------------------------------------------------------------------------------------
// reads a .csv file of the following structure
// first two lines are disregarded, can be used for comments
// remainder is organized like the GUI
// step1, speed1, rest1 for drive1, one empty field, step1 and so on for drive 2 ...
// next line: step2 and so on for drive 1 ...if (this->ardosIsInLoopMode == true) {
// in a better world i will make this routine more elegant ...

void MainWindow::loadTrajectoryFile(void) {
    QString *tfname, *line, *entry;
    double entryNum;

    this->trajectoryFile = new QFile();
    tfname = new QString(QFileDialog::getOpenFileName(this, tr("Trajectory File"), "/home/pi", tr("CSV Files (*.csv)")));
    this->trajectoryFile->setFileName(*tfname);
    this->trajectoryFile->open(QIODevice::ReadOnly);
    this->trajectoryFile->readLine(4096);
    this->trajectoryFile->readLine(4096);   // the first two lines are empty, just for comments
    line = new QString(this->trajectoryFile->readLine(4096));
    entry = new QString();
    entry->append(line->section(",",0,0));
    entryNum = entry->toDouble();
    ui->sbSteps1_1->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",1,1));
    entryNum = entry->toDouble();
    ui->sbSpeed1_1->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",2,2));
    entryNum = entry->toDouble();
    ui->sbCRest1_1->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",4,4));
    entryNum = entry->toDouble();
    ui->sbSteps2_1->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",5,5));
    entryNum = entry->toDouble();
    ui->sbSpeed2_1->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",6,6));
    entryNum = entry->toDouble();
    ui->sbCRest2_1->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",8,8));
    entryNum = entry->toDouble();
    ui->sbSteps3_1->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",9,9));
    entryNum = entry->toDouble();
    ui->sbSpeed3_1->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",10,10));
    entryNum = entry->toDouble();
    ui->sbCRest3_1->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",12,12));
    entryNum = entry->toDouble();
    ui->sbSteps4_1->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",13,13));
    entryNum = entry->toDouble();
    ui->sbSpeed4_1->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",14,14));
    entryNum = entry->toDouble();
    ui->sbCRest4_1->setValue(entryNum);
    entry->clear();
//--- line 1 parsed

    line->clear();
    line->append(this->trajectoryFile->readLine(4096));
    entry->append(line->section(",",0,0));
    entryNum = entry->toDouble();
    ui->sbSteps1_2->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",1,1));
    entryNum = entry->toDouble();
    ui->sbSpeed1_2->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",2,2));
    entryNum = entry->toDouble();
    ui->sbCRest1_2->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",4,4));
    entryNum = entry->toDouble();
    ui->sbSteps2_2->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",5,5));
    entryNum = entry->toDouble();
    ui->sbSpeed2_2->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",6,6));
    entryNum = entry->toDouble();
    ui->sbCRest2_2->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",8,8));
    entryNum = entry->toDouble();
    ui->sbSteps3_2->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",9,9));
    entryNum = entry->toDouble();
    ui->sbSpeed3_2->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",10,10));
    entryNum = entry->toDouble();
    ui->sbCRest3_2->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",12,12));
    entryNum = entry->toDouble();
    ui->sbSteps4_2->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",13,13));
    entryNum = entry->toDouble();
    ui->sbSpeed4_2->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",14,14));
    entryNum = entry->toDouble();
    ui->sbCRest4_2->setValue(entryNum);
    entry->clear();
// ---- line 2 parsed

    line->clear();
    line->append(this->trajectoryFile->readLine(4096));
    entry->append(line->section(",",0,0));
    entryNum = entry->toDouble();
    ui->sbSteps1_3->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",1,1));
    entryNum = entry->toDouble();
    ui->sbSpeed1_3->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",2,2));
    entryNum = entry->toDouble();
    ui->sbCRest1_3->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",4,4));
    entryNum = entry->toDouble();
    ui->sbSteps2_3->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",5,5));
    entryNum = entry->toDouble();
    ui->sbSpeed2_3->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",6,6));
    entryNum = entry->toDouble();
    ui->sbCRest2_3->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",8,8));
    entryNum = entry->toDouble();
    ui->sbSteps3_3->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",9,9));
    entryNum = entry->toDouble();
    ui->sbSpeed3_3->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",10,10));
    entryNum = entry->toDouble();
    ui->sbCRest3_3->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",12,12));
    entryNum = entry->toDouble();
    ui->sbSteps4_3->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",13,13));
    entryNum = entry->toDouble();
    ui->sbSpeed4_3->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",14,14));
    entryNum = entry->toDouble();
    ui->sbCRest4_3->setValue(entryNum);
    entry->clear();
// ---- line 3 parsed

    line->clear();
    line->append(this->trajectoryFile->readLine(4096));
    entry->append(line->section(",",0,0));
    entryNum = entry->toDouble();
    ui->sbSteps1_4->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",1,1));
    entryNum = entry->toDouble();
    ui->sbSpeed1_4->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",2,2));
    entryNum = entry->toDouble();
    ui->sbCRest1_4->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",4,4));
    entryNum = entry->toDouble();
    ui->sbSteps2_4->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",5,5));
    entryNum = entry->toDouble();
    ui->sbSpeed2_4->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",6,6));
    entryNum = entry->toDouble();
    ui->sbCRest2_4->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",8,8));
    entryNum = entry->toDouble();
    ui->sbSteps3_4->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",9,9));
    entryNum = entry->toDouble();
    ui->sbSpeed3_4->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",10,10));
    entryNum = entry->toDouble();
    ui->sbCRest3_4->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",12,12));
    entryNum = entry->toDouble();
    ui->sbSteps4_4->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",13,13));
    entryNum = entry->toDouble();
    ui->sbSpeed4_4->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",14,14));
    entryNum = entry->toDouble();
    ui->sbCRest4_4->setValue(entryNum);
    entry->clear();
// ---- line 4 parsed

    line->clear();
    line->append(this->trajectoryFile->readLine(4096));
    entry->append(line->section(",",0,0));
    entryNum = entry->toDouble();
    ui->sbSteps1_5->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",1,1));
    entryNum = entry->toDouble();
    ui->sbSpeed1_5->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",2,2));
    entryNum = entry->toDouble();
    ui->sbCRest1_5->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",4,4));
    entryNum = entry->toDouble();
    ui->sbSteps2_5->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",5,5));
    entryNum = entry->toDouble();
    ui->sbSpeed2_5->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",6,6));
    entryNum = entry->toDouble();
    ui->sbCRest2_5->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",8,8));
    entryNum = entry->toDouble();
    ui->sbSteps3_5->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",9,9));
    entryNum = entry->toDouble();
    ui->sbSpeed3_5->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",10,10));
    entryNum = entry->toDouble();
    ui->sbCRest3_5->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",12,12));
    entryNum = entry->toDouble();
    ui->sbSteps4_5->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",13,13));
    entryNum = entry->toDouble();
    ui->sbSpeed4_5->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",14,14));
    entryNum = entry->toDouble();
    ui->sbCRest4_5->setValue(entryNum);
    entry->clear();
// ---- line 5 parsed

    line->clear();
    line->append(this->trajectoryFile->readLine(4096));
    entry->append(line->section(",",0,0));
    entryNum = entry->toDouble();
    ui->sbSteps1_6->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",1,1));
    entryNum = entry->toDouble();
    ui->sbSpeed1_6->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",2,2));
    entryNum = entry->toDouble();
    ui->sbCRest1_6->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",4,4));
    entryNum = entry->toDouble();
    ui->sbSteps2_6->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",5,5));
    entryNum = entry->toDouble();
    ui->sbSpeed2_6->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",6,6));
    entryNum = entry->toDouble();
    ui->sbCRest2_6->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",8,8));
    entryNum = entry->toDouble();
    ui->sbSteps3_6->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",9,9));
    entryNum = entry->toDouble();
    ui->sbSpeed3_6->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",10,10));
    entryNum = entry->toDouble();
    ui->sbCRest3_6->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",12,12));
    entryNum = entry->toDouble();
    ui->sbSteps4_6->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",13,13));
    entryNum = entry->toDouble();
    ui->sbSpeed4_6->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",14,14));
    entryNum = entry->toDouble();
    ui->sbCRest4_6->setValue(entryNum);
    entry->clear();
// ---- line 6 parsed

    line->clear();
    line->append(this->trajectoryFile->readLine(4096));
    entry->append(line->section(",",0,0));
    entryNum = entry->toDouble();
    ui->sbSteps1_7->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",1,1));
    entryNum = entry->toDouble();
    ui->sbSpeed1_7->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",2,2));
    entryNum = entry->toDouble();
    ui->sbCRest1_7->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",4,4));
    entryNum = entry->toDouble();
    ui->sbSteps2_7->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",5,5));
    entryNum = entry->toDouble();
    ui->sbSpeed2_7->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",6,6));
    entryNum = entry->toDouble();
    ui->sbCRest2_7->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",8,8));
    entryNum = entry->toDouble();
    ui->sbSteps3_7->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",9,9));
    entryNum = entry->toDouble();
    ui->sbSpeed3_7->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",10,10));
    entryNum = entry->toDouble();
    ui->sbCRest3_7->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",12,12));
    entryNum = entry->toDouble();
    ui->sbSteps4_7->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",13,13));
    entryNum = entry->toDouble();
    ui->sbSpeed4_7->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",14,14));
    entryNum = entry->toDouble();
    ui->sbCRest4_7->setValue(entryNum);
    entry->clear();
// ---- line 7 parsed

    line->clear();
    line->append(this->trajectoryFile->readLine(4096));
    entry->append(line->section(",",0,0));
    entryNum = entry->toDouble();
    ui->sbSteps1_8->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",1,1));
    entryNum = entry->toDouble();
    ui->sbSpeed1_8->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",2,2));
    entryNum = entry->toDouble();
    ui->sbCRest1_8->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",4,4));
    entryNum = entry->toDouble();
    ui->sbSteps2_8->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",5,5));
    entryNum = entry->toDouble();
    ui->sbSpeed2_8->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",6,6));
    entryNum = entry->toDouble();
    ui->sbCRest2_8->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",8,8));
    entryNum = entry->toDouble();
    ui->sbSteps3_8->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",9,9));
    entryNum = entry->toDouble();
    ui->sbSpeed3_8->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",10,10));
    entryNum = entry->toDouble();
    ui->sbCRest3_8->setValue(entryNum);
    entry->clear();

    entry->append(line->section(",",12,12));
    entryNum = entry->toDouble();
    ui->sbSteps4_8->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",13,13));
    entryNum = entry->toDouble();
    ui->sbSpeed4_8->setValue(entryNum);
    entry->clear();
    entry->append(line->section(",",14,14));
    entryNum = entry->toDouble();
    ui->sbCRest4_8->setValue(entryNum);
    entry->clear();
// ---- line 8 parsed

    this->trajectoryFile->close();
    delete line;
    delete tfname;
    conveyTrajectoryToDrive1();
    conveyTrajectoryToDrive2();
    conveyTrajectoryToDrive3();
    conveyTrajectoryToDrive4();
    delete this->trajectoryFile;
}

//---------------------------------------------------------------
void MainWindow::saveTrajectoryFile(void) {
    QString *line,*sfName;
    double entryNum;

    this->trajectoryFile = new QFile();
    sfName= new QString(QFileDialog::getSaveFileName(this, tr("Save Trajectory File as ..."),
                        "/home/pi",tr("CSV-Files (*.csv)")));
    sfName->append(".csv");
    this->trajectoryFile->setFileName(*sfName);
    this->trajectoryFile->open(QIODevice::WriteOnly);
    line = new QString();
    line->append(",,,,,,,,,,,,,,,\n");
    this->trajectoryFile->write(line->toLatin1());
    this->trajectoryFile->write(line->toLatin1());
    line->clear();

    entryNum=ui->sbSteps1_1->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed1_1->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest1_1->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps2_1->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed2_1->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest2_1->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps3_1->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed3_1->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest3_1->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps4_1->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed4_1->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest4_1->value();
    line->append(QString::number(entryNum));
    line->append("\n");
    this->trajectoryFile->write(line->toLatin1());
    line->clear();
//-- line 1 done

    entryNum=ui->sbSteps1_2->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed1_2->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest1_2->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps2_2->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed2_2->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest2_2->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps3_2->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed3_2->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest3_2->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps4_2->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed4_2->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest4_2->value();
    line->append(QString::number(entryNum));
    line->append("\n");
    this->trajectoryFile->write(line->toLatin1());
    line->clear();
//-- line 2 done

    entryNum=ui->sbSteps1_3->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed1_3->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest1_3->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps2_3->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed2_3->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest2_3->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps3_3->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed3_3->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest3_3->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps4_3->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed4_3->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest4_3->value();
    line->append(QString::number(entryNum));
    line->append("\n");
    this->trajectoryFile->write(line->toLatin1());
    line->clear();
//-- line 3 done

    entryNum=ui->sbSteps1_4->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed1_4->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest1_4->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps2_4->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed2_4->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest2_4->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps3_4->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed3_4->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest3_4->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps4_4->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed4_4->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest4_4->value();
    line->append(QString::number(entryNum));
    line->append("\n");
    this->trajectoryFile->write(line->toLatin1());
    line->clear();
//-- line 4 done

    entryNum=ui->sbSteps1_5->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed1_5->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest1_5->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps2_5->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed2_5->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest2_5->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps3_5->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed3_5->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest3_5->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps4_5->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed4_5->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest4_5->value();
    line->append(QString::number(entryNum));
    line->append("\n");
    this->trajectoryFile->write(line->toLatin1());
    line->clear();
//-- line 5 done

    entryNum=ui->sbSteps1_6->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed1_6->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest1_6->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps2_6->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed2_6->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest2_6->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps3_6->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed3_6->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest3_6->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps4_6->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed4_6->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest4_6->value();
    line->append(QString::number(entryNum));
    line->append("\n");
    this->trajectoryFile->write(line->toLatin1());
    line->clear();
//-- line 6 done

    entryNum=ui->sbSteps1_7->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed1_7->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest1_7->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps2_7->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed2_7->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest2_7->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps3_7->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed3_7->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest3_7->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps4_7->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed4_7->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest4_7->value();
    line->append(QString::number(entryNum));
    line->append("\n");
    this->trajectoryFile->write(line->toLatin1());
    line->clear();
//-- line 7 done

    entryNum=ui->sbSteps1_8->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed1_8->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest1_8->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps2_8->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed2_8->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest2_8->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps3_8->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed3_8->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest3_8->value();
    line->append(QString::number(entryNum));
    line->append(",,");
    entryNum=ui->sbSteps4_8->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbSpeed4_8->value();
    line->append(QString::number(entryNum));
    line->append(",");
    entryNum=ui->sbCRest4_8->value();
    line->append(QString::number(entryNum));
    line->append("\n");
    this->trajectoryFile->write(line->toLatin1());
    line->clear();
//-- line 8 done

    this->trajectoryFile->close();
    delete line;
    delete this->trajectoryFile;
    delete sfName;
}
