#ifndef _LINUX_GAF_REC_H
#define _LINUX_GAF_REC_H

//{{ ss.sec
/*
 * Recorder for GAF v3.0
 * This file contains the cod used by event recording routines. *
 */

#define NUM_OF_CPUS num_online_cpus()
#define NUM_OF_COLUMNS_0 24
#define NUM_OF_LOWS_0 1024  // it must be 2^n

#pragma pack(4)   // because of alignment

struct GAF_RECORD_0 {
	int version;  //version 1.0
	int lows;
	int columns;
	char* column_title[NUM_OF_COLUMNS_0];
	int save_index[NUM_OF_CPUS];
	struct COLUMN {
		unsigned long long time;
		unsigned int pid;
		unsigned int data[NUM_OF_COLUMNS_0 - 2];
	} column[NUM_OF_CPUS][NUM_OF_LOWS_0];
};
#pragma pack()


#endif /* _LINUX_GAF_REC_H */
