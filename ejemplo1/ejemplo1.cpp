#include "ejemplo1.h"



ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
	connect(pushButton, SIGNAL(clicked()), this, SLOT(doReset()) );
	connect(&timer, SIGNAL(timeout()), this, SLOT(count()) );
	connect(horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(doSlider(int)) );
	value = 0;
	timer.start(500);

}

void ejemplo1::doButton()
{
	qDebug() << "click on button";
	timer.stop();
}

void ejemplo1::count()
{
	lcdNumber->display(value++);
}

void ejemplo1::doSlider(int val)
{
	timer.setInterval(val);
	lcdNumber_2->display(val);
}

void ejemplo1::doReset()
{
	value = 0;
}




