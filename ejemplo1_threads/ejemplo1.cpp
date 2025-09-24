#include "ejemplo1.h"



ejemplo1::ejemplo1(): Ui_Counter()
{

	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
	connect(pushButton, SIGNAL(clicked()), this, SLOT(doReset()) );
	timer.connect(std::bind(&ejemplo1::count, this, 1));
	connect(horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(doSlider(int)) );
	value = 0;
	timer.start(500);

}

void ejemplo1::doButton()
{
	qDebug() << "click on button";
	timer.stop();
}

void ejemplo1::count(int i)
{
	printf("Bind: %d ", i);
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




