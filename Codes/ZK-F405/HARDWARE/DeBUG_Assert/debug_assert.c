
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"

#include "debug_assert.h"
#include "led.h"
//#include "motors.h"



#define MAGIC_ASSERT_INDICATOR 0x2f8a001f

typedef struct SNAPSHOT_DATA 
{
	u32 magicNumber;
	char* fileName;
	int line;
} SNAPSHOT_DATA;

// The .nzds section is not cleared at startup, data here will survive a
// reset (by the watch dog for instance)
SNAPSHOT_DATA snapshot __attribute__((section(".nzds"))) = 
{
	.magicNumber = 0,
	.fileName = "",
	.line = 0
};


void assertFail(char *exp, char *file, int line)
{
	portDISABLE_INTERRUPTS();
	storeAssertSnapshotData(file, line);
	printf("Assert failed %s:%d\n", file, line);
	while (1);
}

void storeAssertSnapshotData(char *file, int line)
{
	snapshot.magicNumber = MAGIC_ASSERT_INDICATOR;
	snapshot.fileName = file;
	snapshot.line = line;
}

void printAssertSnapshotData()
{
	if (MAGIC_ASSERT_INDICATOR == snapshot.magicNumber) 
	{
		printf("Assert failed at %s:%d\n", snapshot.fileName, snapshot.line);
	} else 
	{
		printf("No assert information found\n");
	}
}


