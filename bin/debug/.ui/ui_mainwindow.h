/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *btnFindNextPose;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QLabel *lblPenaltyTolerance;
    QLabel *lblAngleTolerance;
    QLabel *lblMinimumLength;
    QLabel *lblJoinBoundariesDistance;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_2;
    QDoubleSpinBox *dsbPenaltyTolerance;
    QDoubleSpinBox *dsbAngleTolerance;
    QDoubleSpinBox *dsbMinimumLength;
    QDoubleSpinBox *dsbJoinBoundariesDistance;
    QLabel *ToFindBoundaries;
    QWidget *verticalLayoutWidget_3;
    QVBoxLayout *verticalLayout_3;
    QLabel *lblChainSmallRadius;
    QLabel *lblChainBigRadius;
    QLabel *lblChainIncRadius;
    QLabel *lblChainDefaultBigRadius;
    QLabel *lblChainMaxBigRadius;
    QWidget *verticalLayoutWidget_4;
    QVBoxLayout *verticalLayout_4;
    QDoubleSpinBox *dsbChainSmallRadius;
    QDoubleSpinBox *dsbChainBigRadius;
    QDoubleSpinBox *dsbChainIncRadius;
    QDoubleSpinBox *dsbChainDefaultBigRadius;
    QDoubleSpinBox *dsbChainMaxBigRadius;
    QLabel *lblToFindChains;
    QWidget *verticalLayoutWidget_5;
    QVBoxLayout *verticalLayout_5;
    QDoubleSpinBox *dsbCameraDistance;
    QDoubleSpinBox *dsbOverlappingCoefficient;
    QDoubleSpinBox *dsbCameraAngleToCoeff;
    QWidget *verticalLayoutWidget_6;
    QVBoxLayout *verticalLayout_6;
    QLabel *lblCameraDistance;
    QLabel *lblOverlappingCoefficient;
    QLabel *lblCameraAngleToCoeff;
    QFrame *line;
    QFrame *line_2;
    QFrame *line_3;
    QLabel *ToFindBoundaries_2;
    QFrame *line_4;
    QLabel *lblFileName;
    QTextEdit *txtFileName;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(579, 331);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        btnFindNextPose = new QPushButton(centralWidget);
        btnFindNextPose->setObjectName(QStringLiteral("btnFindNextPose"));
        btnFindNextPose->setGeometry(QRect(399, 260, 101, 23));
        verticalLayoutWidget = new QWidget(centralWidget);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(20, 30, 152, 121));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        lblPenaltyTolerance = new QLabel(verticalLayoutWidget);
        lblPenaltyTolerance->setObjectName(QStringLiteral("lblPenaltyTolerance"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(lblPenaltyTolerance->sizePolicy().hasHeightForWidth());
        lblPenaltyTolerance->setSizePolicy(sizePolicy);

        verticalLayout->addWidget(lblPenaltyTolerance);

        lblAngleTolerance = new QLabel(verticalLayoutWidget);
        lblAngleTolerance->setObjectName(QStringLiteral("lblAngleTolerance"));
        sizePolicy.setHeightForWidth(lblAngleTolerance->sizePolicy().hasHeightForWidth());
        lblAngleTolerance->setSizePolicy(sizePolicy);

        verticalLayout->addWidget(lblAngleTolerance);

        lblMinimumLength = new QLabel(verticalLayoutWidget);
        lblMinimumLength->setObjectName(QStringLiteral("lblMinimumLength"));
        sizePolicy.setHeightForWidth(lblMinimumLength->sizePolicy().hasHeightForWidth());
        lblMinimumLength->setSizePolicy(sizePolicy);

        verticalLayout->addWidget(lblMinimumLength);

        lblJoinBoundariesDistance = new QLabel(verticalLayoutWidget);
        lblJoinBoundariesDistance->setObjectName(QStringLiteral("lblJoinBoundariesDistance"));
        sizePolicy.setHeightForWidth(lblJoinBoundariesDistance->sizePolicy().hasHeightForWidth());
        lblJoinBoundariesDistance->setSizePolicy(sizePolicy);

        verticalLayout->addWidget(lblJoinBoundariesDistance);

        verticalLayoutWidget_2 = new QWidget(centralWidget);
        verticalLayoutWidget_2->setObjectName(QStringLiteral("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(200, 30, 91, 121));
        verticalLayout_2 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        dsbPenaltyTolerance = new QDoubleSpinBox(verticalLayoutWidget_2);
        dsbPenaltyTolerance->setObjectName(QStringLiteral("dsbPenaltyTolerance"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(dsbPenaltyTolerance->sizePolicy().hasHeightForWidth());
        dsbPenaltyTolerance->setSizePolicy(sizePolicy1);
        dsbPenaltyTolerance->setSingleStep(0.05);

        verticalLayout_2->addWidget(dsbPenaltyTolerance);

        dsbAngleTolerance = new QDoubleSpinBox(verticalLayoutWidget_2);
        dsbAngleTolerance->setObjectName(QStringLiteral("dsbAngleTolerance"));
        sizePolicy1.setHeightForWidth(dsbAngleTolerance->sizePolicy().hasHeightForWidth());
        dsbAngleTolerance->setSizePolicy(sizePolicy1);

        verticalLayout_2->addWidget(dsbAngleTolerance);

        dsbMinimumLength = new QDoubleSpinBox(verticalLayoutWidget_2);
        dsbMinimumLength->setObjectName(QStringLiteral("dsbMinimumLength"));
        sizePolicy1.setHeightForWidth(dsbMinimumLength->sizePolicy().hasHeightForWidth());
        dsbMinimumLength->setSizePolicy(sizePolicy1);

        verticalLayout_2->addWidget(dsbMinimumLength);

        dsbJoinBoundariesDistance = new QDoubleSpinBox(verticalLayoutWidget_2);
        dsbJoinBoundariesDistance->setObjectName(QStringLiteral("dsbJoinBoundariesDistance"));
        sizePolicy1.setHeightForWidth(dsbJoinBoundariesDistance->sizePolicy().hasHeightForWidth());
        dsbJoinBoundariesDistance->setSizePolicy(sizePolicy1);

        verticalLayout_2->addWidget(dsbJoinBoundariesDistance);

        ToFindBoundaries = new QLabel(centralWidget);
        ToFindBoundaries->setObjectName(QStringLiteral("ToFindBoundaries"));
        ToFindBoundaries->setGeometry(QRect(20, 10, 111, 16));
        verticalLayoutWidget_3 = new QWidget(centralWidget);
        verticalLayoutWidget_3->setObjectName(QStringLiteral("verticalLayoutWidget_3"));
        verticalLayoutWidget_3->setGeometry(QRect(310, 30, 151, 151));
        verticalLayout_3 = new QVBoxLayout(verticalLayoutWidget_3);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        lblChainSmallRadius = new QLabel(verticalLayoutWidget_3);
        lblChainSmallRadius->setObjectName(QStringLiteral("lblChainSmallRadius"));
        sizePolicy.setHeightForWidth(lblChainSmallRadius->sizePolicy().hasHeightForWidth());
        lblChainSmallRadius->setSizePolicy(sizePolicy);

        verticalLayout_3->addWidget(lblChainSmallRadius);

        lblChainBigRadius = new QLabel(verticalLayoutWidget_3);
        lblChainBigRadius->setObjectName(QStringLiteral("lblChainBigRadius"));
        sizePolicy.setHeightForWidth(lblChainBigRadius->sizePolicy().hasHeightForWidth());
        lblChainBigRadius->setSizePolicy(sizePolicy);

        verticalLayout_3->addWidget(lblChainBigRadius);

        lblChainIncRadius = new QLabel(verticalLayoutWidget_3);
        lblChainIncRadius->setObjectName(QStringLiteral("lblChainIncRadius"));
        sizePolicy.setHeightForWidth(lblChainIncRadius->sizePolicy().hasHeightForWidth());
        lblChainIncRadius->setSizePolicy(sizePolicy);

        verticalLayout_3->addWidget(lblChainIncRadius);

        lblChainDefaultBigRadius = new QLabel(verticalLayoutWidget_3);
        lblChainDefaultBigRadius->setObjectName(QStringLiteral("lblChainDefaultBigRadius"));
        sizePolicy.setHeightForWidth(lblChainDefaultBigRadius->sizePolicy().hasHeightForWidth());
        lblChainDefaultBigRadius->setSizePolicy(sizePolicy);

        verticalLayout_3->addWidget(lblChainDefaultBigRadius);

        lblChainMaxBigRadius = new QLabel(verticalLayoutWidget_3);
        lblChainMaxBigRadius->setObjectName(QStringLiteral("lblChainMaxBigRadius"));
        sizePolicy.setHeightForWidth(lblChainMaxBigRadius->sizePolicy().hasHeightForWidth());
        lblChainMaxBigRadius->setSizePolicy(sizePolicy);

        verticalLayout_3->addWidget(lblChainMaxBigRadius);

        verticalLayoutWidget_4 = new QWidget(centralWidget);
        verticalLayoutWidget_4->setObjectName(QStringLiteral("verticalLayoutWidget_4"));
        verticalLayoutWidget_4->setGeometry(QRect(470, 30, 91, 151));
        verticalLayout_4 = new QVBoxLayout(verticalLayoutWidget_4);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        dsbChainSmallRadius = new QDoubleSpinBox(verticalLayoutWidget_4);
        dsbChainSmallRadius->setObjectName(QStringLiteral("dsbChainSmallRadius"));
        sizePolicy1.setHeightForWidth(dsbChainSmallRadius->sizePolicy().hasHeightForWidth());
        dsbChainSmallRadius->setSizePolicy(sizePolicy1);
        dsbChainSmallRadius->setDecimals(3);
        dsbChainSmallRadius->setSingleStep(0.001);

        verticalLayout_4->addWidget(dsbChainSmallRadius);

        dsbChainBigRadius = new QDoubleSpinBox(verticalLayoutWidget_4);
        dsbChainBigRadius->setObjectName(QStringLiteral("dsbChainBigRadius"));
        sizePolicy1.setHeightForWidth(dsbChainBigRadius->sizePolicy().hasHeightForWidth());
        dsbChainBigRadius->setSizePolicy(sizePolicy1);
        dsbChainBigRadius->setDecimals(3);
        dsbChainBigRadius->setSingleStep(0.001);

        verticalLayout_4->addWidget(dsbChainBigRadius);

        dsbChainIncRadius = new QDoubleSpinBox(verticalLayoutWidget_4);
        dsbChainIncRadius->setObjectName(QStringLiteral("dsbChainIncRadius"));
        sizePolicy1.setHeightForWidth(dsbChainIncRadius->sizePolicy().hasHeightForWidth());
        dsbChainIncRadius->setSizePolicy(sizePolicy1);
        dsbChainIncRadius->setDecimals(3);
        dsbChainIncRadius->setSingleStep(0.001);

        verticalLayout_4->addWidget(dsbChainIncRadius);

        dsbChainDefaultBigRadius = new QDoubleSpinBox(verticalLayoutWidget_4);
        dsbChainDefaultBigRadius->setObjectName(QStringLiteral("dsbChainDefaultBigRadius"));
        sizePolicy1.setHeightForWidth(dsbChainDefaultBigRadius->sizePolicy().hasHeightForWidth());
        dsbChainDefaultBigRadius->setSizePolicy(sizePolicy1);
        dsbChainDefaultBigRadius->setDecimals(3);
        dsbChainDefaultBigRadius->setSingleStep(0.001);

        verticalLayout_4->addWidget(dsbChainDefaultBigRadius);

        dsbChainMaxBigRadius = new QDoubleSpinBox(verticalLayoutWidget_4);
        dsbChainMaxBigRadius->setObjectName(QStringLiteral("dsbChainMaxBigRadius"));
        sizePolicy1.setHeightForWidth(dsbChainMaxBigRadius->sizePolicy().hasHeightForWidth());
        dsbChainMaxBigRadius->setSizePolicy(sizePolicy1);
        dsbChainMaxBigRadius->setDecimals(3);
        dsbChainMaxBigRadius->setSingleStep(0.001);

        verticalLayout_4->addWidget(dsbChainMaxBigRadius);

        lblToFindChains = new QLabel(centralWidget);
        lblToFindChains->setObjectName(QStringLiteral("lblToFindChains"));
        lblToFindChains->setGeometry(QRect(310, 10, 111, 16));
        verticalLayoutWidget_5 = new QWidget(centralWidget);
        verticalLayoutWidget_5->setObjectName(QStringLiteral("verticalLayoutWidget_5"));
        verticalLayoutWidget_5->setGeometry(QRect(200, 200, 81, 91));
        verticalLayout_5 = new QVBoxLayout(verticalLayoutWidget_5);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        dsbCameraDistance = new QDoubleSpinBox(verticalLayoutWidget_5);
        dsbCameraDistance->setObjectName(QStringLiteral("dsbCameraDistance"));
        sizePolicy1.setHeightForWidth(dsbCameraDistance->sizePolicy().hasHeightForWidth());
        dsbCameraDistance->setSizePolicy(sizePolicy1);
        dsbCameraDistance->setSingleStep(0.05);

        verticalLayout_5->addWidget(dsbCameraDistance);

        dsbOverlappingCoefficient = new QDoubleSpinBox(verticalLayoutWidget_5);
        dsbOverlappingCoefficient->setObjectName(QStringLiteral("dsbOverlappingCoefficient"));
        sizePolicy1.setHeightForWidth(dsbOverlappingCoefficient->sizePolicy().hasHeightForWidth());
        dsbOverlappingCoefficient->setSizePolicy(sizePolicy1);

        verticalLayout_5->addWidget(dsbOverlappingCoefficient);

        dsbCameraAngleToCoeff = new QDoubleSpinBox(verticalLayoutWidget_5);
        dsbCameraAngleToCoeff->setObjectName(QStringLiteral("dsbCameraAngleToCoeff"));
        sizePolicy1.setHeightForWidth(dsbCameraAngleToCoeff->sizePolicy().hasHeightForWidth());
        dsbCameraAngleToCoeff->setSizePolicy(sizePolicy1);

        verticalLayout_5->addWidget(dsbCameraAngleToCoeff);

        verticalLayoutWidget_6 = new QWidget(centralWidget);
        verticalLayoutWidget_6->setObjectName(QStringLiteral("verticalLayoutWidget_6"));
        verticalLayoutWidget_6->setGeometry(QRect(20, 200, 181, 91));
        verticalLayout_6 = new QVBoxLayout(verticalLayoutWidget_6);
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setContentsMargins(11, 11, 11, 11);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        verticalLayout_6->setContentsMargins(0, 0, 0, 0);
        lblCameraDistance = new QLabel(verticalLayoutWidget_6);
        lblCameraDistance->setObjectName(QStringLiteral("lblCameraDistance"));
        sizePolicy.setHeightForWidth(lblCameraDistance->sizePolicy().hasHeightForWidth());
        lblCameraDistance->setSizePolicy(sizePolicy);

        verticalLayout_6->addWidget(lblCameraDistance);

        lblOverlappingCoefficient = new QLabel(verticalLayoutWidget_6);
        lblOverlappingCoefficient->setObjectName(QStringLiteral("lblOverlappingCoefficient"));
        sizePolicy.setHeightForWidth(lblOverlappingCoefficient->sizePolicy().hasHeightForWidth());
        lblOverlappingCoefficient->setSizePolicy(sizePolicy);

        verticalLayout_6->addWidget(lblOverlappingCoefficient);

        lblCameraAngleToCoeff = new QLabel(verticalLayoutWidget_6);
        lblCameraAngleToCoeff->setObjectName(QStringLiteral("lblCameraAngleToCoeff"));
        sizePolicy.setHeightForWidth(lblCameraAngleToCoeff->sizePolicy().hasHeightForWidth());
        lblCameraAngleToCoeff->setSizePolicy(sizePolicy);

        verticalLayout_6->addWidget(lblCameraAngleToCoeff);

        line = new QFrame(centralWidget);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(283, 0, 31, 291));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        line_2 = new QFrame(centralWidget);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setGeometry(QRect(10, 20, 271, 16));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        line_3 = new QFrame(centralWidget);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setGeometry(QRect(300, 20, 271, 16));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        ToFindBoundaries_2 = new QLabel(centralWidget);
        ToFindBoundaries_2->setObjectName(QStringLiteral("ToFindBoundaries_2"));
        ToFindBoundaries_2->setGeometry(QRect(20, 180, 201, 16));
        line_4 = new QFrame(centralWidget);
        line_4->setObjectName(QStringLiteral("line_4"));
        line_4->setGeometry(QRect(10, 190, 271, 16));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);
        lblFileName = new QLabel(centralWidget);
        lblFileName->setObjectName(QStringLiteral("lblFileName"));
        lblFileName->setGeometry(QRect(310, 220, 61, 21));
        txtFileName = new QTextEdit(centralWidget);
        txtFileName->setObjectName(QStringLiteral("txtFileName"));
        txtFileName->setGeometry(QRect(380, 220, 161, 21));
        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        btnFindNextPose->setText(QApplication::translate("MainWindow", "Find Next Pose", 0));
        lblPenaltyTolerance->setText(QApplication::translate("MainWindow", "Penalty Tolerance", 0));
        lblAngleTolerance->setText(QApplication::translate("MainWindow", "Angle Tolerance", 0));
        lblMinimumLength->setText(QApplication::translate("MainWindow", "Minimium Length", 0));
        lblJoinBoundariesDistance->setText(QApplication::translate("MainWindow", "Join Boundaries Distance", 0));
        ToFindBoundaries->setText(QApplication::translate("MainWindow", "To find Boundaries", 0));
        lblChainSmallRadius->setText(QApplication::translate("MainWindow", "Small Radius", 0));
        lblChainBigRadius->setText(QApplication::translate("MainWindow", "Big Radius", 0));
        lblChainIncRadius->setText(QApplication::translate("MainWindow", "Increment to Radius", 0));
        lblChainDefaultBigRadius->setText(QApplication::translate("MainWindow", "Default Big Radius", 0));
        lblChainMaxBigRadius->setText(QApplication::translate("MainWindow", "Maximum Big Radius", 0));
        lblToFindChains->setText(QApplication::translate("MainWindow", "To find chains", 0));
        lblCameraDistance->setText(QApplication::translate("MainWindow", "Camera Distance", 0));
        lblOverlappingCoefficient->setText(QApplication::translate("MainWindow", "Overlapping Coefficient", 0));
        lblCameraAngleToCoeff->setText(QApplication::translate("MainWindow", "Camera Angle to Coefficient", 0));
        ToFindBoundaries_2->setText(QApplication::translate("MainWindow", "Next Pose Distance from Object", 0));
        lblFileName->setText(QApplication::translate("MainWindow", "File Name", 0));
        txtFileName->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">cloud0</p></body></html>", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
