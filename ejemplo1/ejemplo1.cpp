#include "ejemplo1.h"
#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
	connect(&timer, SIGNAL(timeout()), this, SLOT(count()) );
	connect(&slider, SIGNAL(sliderMoved()), this, SLOT(doSlider()) );
	timer.start(500);

}

void ejemplo1::doButton()
{

	qDebug() << "click on button";
	timer.stop();

}

void ejemplo1::count()
{
	static int value = 0;
	lcdNumber->display(value++);
}

void ejemplo1::doSlider()
{
	timer.setInterval(200000000000);
}



