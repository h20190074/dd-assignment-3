#include<linux/kernel.h>
#include<linux/module.h>
#include<linux/usb.h>
#include <linux/slab.h>
#include<linux/types.h>
#include <linux/fs.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/errno.h>
#include<linux/workqueue.h>
#include<linux/sched.h>


#define CARD_READER_VID 0xaaaa    //card reader MXT USB Device          
#define CARD_READER_PID  0x8816                

#define SONY_VID  0x054c
#define SONY_PID  0x05ba

#define SANDISK_VID 0x0781
#define SANDISK_PID 0x1303

#define GET_MAX_LUN        0xFE
#define REQ_TYPE		   0xA1
#define RESET_TYPE         0x21
#define RESET_D			   0xFF 

#define READ_CAPACITY_LENGTH          0x08
#define READ_INQ_LENGTH          0x24
#define READ_DATA               0X200

#define CBW_LEN 31
#define CSW_LEN 13

/* 
 * The internal structure representation of our Device
 */
static struct workqueue_struct *myqueue = NULL; 
struct my_work{
	struct work_struct work;
	void *data;
}; 
 
static struct rb_device
{
	/* Size is the size of the device (in sectors) */
	unsigned int size;
	/* For exclusive access to our request queue */
	spinlock_t lock;
	/* Our request queue */
	struct request_queue *rb_queue;
	/* This is kernel's representation of an individual disk device */
	struct gendisk *rb_disk;
} rb_dev;


static u32 tag = 1;
static u_int rb_major = 0;
static u32  max_lba,block_size1;  //maximum logical block adress and block size
u8 lun;   //max lun
u8 cdb[16]; //CBD
u32 exp_tag; //expected return tag
u8 endpoint_in_addr,endpoint_out_addr; //endpoint adress for bulk only devices

struct usb_device *udev;  //A pointer to the USB device that the descriptor should be retrieved from 

//to determine length of scsi command

static u8 cdb_length[256] = {
//	 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  0
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  1
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  2
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  3
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  4
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  5
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  6
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  7
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  8
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  9
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  A
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  B
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  C
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  D
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  E
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  F
};

//supported device table
static struct usb_device_id usbdev_table [] = {
	{USB_DEVICE(CARD_READER_VID, CARD_READER_PID)},
	{USB_DEVICE(SONY_VID, SONY_PID)},
	{USB_DEVICE(SANDISK_VID, SANDISK_PID)},
	{} /*terminating entry*/	
};


// Command Block Wrapper (CBW)
struct command_block_wrapper {
	u8 dCBWSignature[4];
	u32 dCBWTag;
	u32 dCBWDataTransferLength;
	u8 bmCBWFlags;
	u8 bCBWLUN;
	u8 bCBWCBLength;
	u8 CBWCB[16];
};


// Command Status Wrapper (CSW)
struct command_status_wrapper {
	u8 dCSWSignature[4];
	u32 dCSWTag;
	u32 dCSWDataResidue;
	u8 bCSWStatus;
};


//function prototype
int find_lun(void);
int only_inq(void);
int only_cap(void);
int only_read(sector_t sector_off, u8 *buffer, unsigned int sectors);
int only_write(sector_t sector_off, u8 *buffer, unsigned int sectors);
int get_status(void);
int reset_dev(void);
int initblock(void);


//function defenisions


//------------get status of received data----------//
int get_status()
{	
	int size,k;
	struct command_status_wrapper csw;
	memset(&csw, 0, sizeof(csw));
	printk(KERN_INFO "checking status\n");
	
	k=usb_bulk_msg(udev, usb_rcvbulkpipe (udev,endpoint_in_addr), (unsigned char*)&csw, CSW_LEN, &size, 3000);
	if(k!=0)
	{
		return k;
	}
	if(csw.bCSWStatus!=0)
	{
		printk(KERN_INFO "	status: failed\n");
		return -1;
	}
	if(csw.dCSWTag!=exp_tag) 
	{
		printk(KERN_INFO "tag not matched\n");
		return -1;
	}
	if(csw.dCSWSignature[0]!=85 || csw.dCSWSignature[1]!=83 || csw.dCSWSignature[2]!= 66 || csw.dCSWSignature[3]!=83)
	{
		printk(KERN_INFO "	signalture not matched\n");
		return -1;
	}
	printk(KERN_INFO "   status sucess\n");
	printk("   tag=%u, status=%u, signature=%x%x%x%x\n",csw.dCSWTag,csw.bCSWStatus,csw.dCSWSignature[0],csw.dCSWSignature[1],csw.dCSWSignature[2],csw.dCSWSignature[3]);
	return 0;

}



/*------------resetting device---------------*/

int reset_dev(){
int r;   //number of bytes receivedd after control msg sent or error
void *lun1= kmalloc(1, GFP_KERNEL);

printk(KERN_INFO "sending control messege to reset\n");
r=usb_control_msg(udev, usb_rcvctrlpipe (udev,0x00),RESET_D,RESET_TYPE, 0, 0, lun1, 0, 10000);  //sending control signal
memcpy(&lun, lun1, 1);
kfree(lun1);
printk("----------------------------\n");

if(r==0)   //for stall
{
lun=0;
}

if(r<0)  //for error
{
return r;
}

printk("   received byte after control signal sent=%d, Max lun got as: %u \n",r,lun);  

printk("----------------------------\n");

return 0;
}


/*-----------finding maximum lun------------*/

int find_lun()
{
int r;   //number of bytes receivedd after control msg sent or error
void *lun1= kmalloc(1, GFP_KERNEL);

printk(KERN_INFO "sending control messege to get max LUN\n");
r=usb_control_msg(udev, usb_rcvctrlpipe (udev,0x00),GET_MAX_LUN,REQ_TYPE, 0, 0, lun1, 1, 10000);  //sending control signal
memcpy(&lun, lun1, 1);
kfree(lun1);
printk("----------------------------\n");

if(r==0)   //for stall
{
lun=0;
}

if(r<0)  //for error
{
return r;
}

printk("   received byte after control signal sent=%d, Max lun got as: %u \n",r,lun);  

printk("----------------------------\n");

return 0;
}
//-----------------only inquery-------------------------------//

int only_inq()
{
	u8 buffer[64];  //buffer to store inquery data
	unsigned char vid[9], pid[9], rev[5];            //to  veryfy PID and VID through SCSI command
	int ei,ej;         //error codes for: inquiry send , inquiry recieve
	int i,k;			 //error codes for: bulk send , bulk recieve, stauts
	u8 cdb_len;          //CBWCB length
	int size;
	//static u32 tag = 1;


	struct command_block_wrapper cbw;
	memset(&cbw, 0, sizeof(cbw));
	memset(cdb, 0, sizeof(cdb));
	memset(buffer, 0, sizeof(buffer));

   cbw.dCBWSignature[0] = 'U';
	cbw.dCBWSignature[1] = 'S';
	cbw.dCBWSignature[2] = 'B';
	cbw.dCBWSignature[3] = 'C';	
	cbw.bCBWLUN = lun;
	cbw.bmCBWFlags = USB_DIR_IN; //since data transfer is in for both data Inquiry and capacity- 0x80

	cdb[0] = 0x12;	// Inquiry
	cdb[4] = 0x24;
	cdb_len = cdb_length[cdb[0]];
	cbw.dCBWTag = tag;
	exp_tag=tag;
	cbw.dCBWDataTransferLength = READ_INQ_LENGTH; //
 	cbw.bCBWCBLength = cdb_len;
	memcpy(cbw.CBWCB, cdb, cdb_len);
	exp_tag=tag;
	

	//--------inqury send---------//
	printk(KERN_INFO "Sending inquiry command\n");
	ei=usb_bulk_msg(udev, usb_sndbulkpipe(udev,endpoint_out_addr),(unsigned char*)&cbw, 31, &size, 10000);
	if(ei!=0)
	{
	return ei;
	}
	printk(KERN_INFO "   sent %d CDB bytes and sent %d CBW bytes\n", cdb_len,size);
	//--------Inquiry receive----//
	ej=usb_bulk_msg(udev, usb_rcvbulkpipe(udev,endpoint_in_addr), (unsigned char*)&buffer, READ_INQ_LENGTH, &size, 1000);
	printk(KERN_ALERT "   received inquiry %d bytes\n", size);
	if(ej!=0)
	{
	return ej;
	}
	//---------get status----------//
	k=get_status();
	if(k==-1)
	{
		return -1;
	}

	for (i=0; i<8; i++) {
		vid[i] = buffer[8+i];
		pid[i] = buffer[16+i];
		rev[i/2] = buffer[32+i/2];	// instead of another loop
	}

	vid[8] = 0;
	pid[8] = 0;
	rev[4] = 0;
	printk(KERN_ALERT "	after inquiry   VID:PID:REV \"%8s\":\"%8s\":\"%4s\"\n", vid, pid, rev); 
	printk("----------------------------\n");
	return 0;
}

//----------------only capacity determination----------------------//

int only_cap()
{

	u8 buffer1[8];  //buffer to store capacity data
	u64 big,temp1,temp2,temp3;  //for capacity calculation
	int i,r,k;			 //error codes for: bulk send , bulk recieve, stauts
	int size,size1;  //received sizes
	u8 cdb_len;          //CBWCB length 
	//static u32 tag = 2;
	int ct=0;   //incase of stall 
	struct command_block_wrapper cbw;
	memset(&cbw, 0, sizeof(cbw));
	memset(cdb, 0, sizeof(cdb));
	memset(buffer1, 0, sizeof(buffer1));
	
	cbw.dCBWSignature[0] = 'U';
	cbw.dCBWSignature[1] = 'S';
	cbw.dCBWSignature[2] = 'B';
	cbw.dCBWSignature[3] = 'C';	
	cbw.bCBWLUN = lun;
	cbw.bmCBWFlags = USB_DIR_IN; //since data transfer is in for both data Inquiry and capacity- 0x80

	cdb[0] = 0x25;
	tag++;
	cbw.dCBWTag = tag;
	exp_tag=tag;
	cbw.dCBWDataTransferLength = READ_CAPACITY_LENGTH; //
	cdb_len = cdb_length[cdb[0]];	
	cbw.bCBWCBLength = cdb_len;
	memcpy(cbw.CBWCB, cdb, cdb_len);


	// The transfer length must always be exactly 31 bytes.

	//--------capacity  send---------//
	printk(KERN_INFO "sending capacity read command\n");
	r = usb_bulk_msg(udev, usb_sndbulkpipe(udev,endpoint_out_addr),(unsigned char*)&cbw, 31, &size, 10000);

   while(r!=0 && ct<5)  //if not sent first time
	{
		r=usb_clear_halt(udev,usb_sndbulkpipe(udev,endpoint_out_addr)) ;

		r = usb_bulk_msg(udev, usb_sndbulkpipe(udev,endpoint_out_addr),(unsigned char*)&cbw, 31, &size, 1000);
		ct++;

	}
	if(ct==5 && r!=0)
	{
		ct=0;
		return r;
	}

	printk(KERN_INFO "   sent %d CDB bytes and sent %d CBW bytes\n", cdb_len,size);

	//--------capacity  recieve---------//

	i=usb_bulk_msg(udev, usb_rcvbulkpipe (udev,endpoint_in_addr), (unsigned char*)&buffer1,  READ_CAPACITY_LENGTH, &size1, 10000);
	printk(KERN_INFO "   received %d bytes\n", size1);
	if(i!=0)
	{
		return i;
	}
	k=get_status();  //check status
	if(k==-1)
	{
		return -1;
	}



	max_lba= (buffer1[0] << 24) | (buffer1[1] << 16) | ( buffer1[2] << 8 ) | (buffer1[3]);   //get max logical adress
	block_size1=(buffer1[4] << 24) | (buffer1[5] << 16) | ( buffer1[6] << 8 ) | (buffer1[7]); //get block size
	big=(u64)(max_lba+1)*(u64)(block_size1); //total memeory size in bytes
	temp1=big>>30;
	temp2=big%(1<<30);
	temp3=(temp2*10)>>30; //total memory size in GB
	printk(KERN_INFO "capacity of device is:\n");
	printk(KERN_INFO "	in hex:   Max LBA: %08X, Block Size: %08X \n	therefore total size: (%llx Bytes)\n", max_lba, block_size1, big);
	printk(KERN_INFO "	in decimal:   Max LBA: %u, Block Size: %u Bytes\n	therefore total size: (%lu Bytes) (%lu.%lu GB)\n", (unsigned int)max_lba,(unsigned int)block_size1,(unsigned long) big,(unsigned long)temp1,(unsigned long)temp3);

	printk("----------------------------\n");
	return 0;

}


//-----------------only read ----------------------------------//
int only_read(sector_t sector_off, u8 *buffer, unsigned int sectors)
{
	//u8 buffer2[512];
	unsigned int hi;
	int i,r,k,r1;			 //error codes for: bulk send , bulk recieve, stauts
	int size2,size3;  //received sizes
	u8 cdb_len;          //CBWCB length
	//static u32 tag = 3;                                                           //make it globel later
	int ct=0;   //incase of stall 
	int pooja;
	struct command_block_wrapper cbw;
	memset(&cbw, 0, sizeof(cbw));
	memset(cdb, 0, sizeof(cdb));
	//memset(buffer2, 0, sizeof(buffer2));
	r=0;
	cbw.dCBWSignature[0] = 'U';
	cbw.dCBWSignature[1] = 'S';
	cbw.dCBWSignature[2] = 'B';
	cbw.dCBWSignature[3] = 'C';	
	cbw.bCBWLUN = lun;
	cbw.bmCBWFlags = USB_DIR_IN; //since data transfer is in for both data Inquiry and capacity- 0x80
	cdb[0] = 0x28;
	cdb[1] = 0x00;
	cdb[2] = (sector_off & 0xFF000000) >> 24;
	cdb[3] = (sector_off & 0xFF0000) >> 16;
	cdb[4] = (sector_off & 0xFF00) >> 8;
	cdb[5] = (sector_off & 0xFF);
	cdb[7] = ((u16)sectors & 0xff00);
	cdb[8] = ((u16)sectors & 0x00ff);
	tag++;
	cbw.dCBWTag = tag;
	exp_tag=tag;
	hi=READ_DATA*sectors;
	cbw.dCBWDataTransferLength =hi ; //
	cdb_len = cdb_length[cdb[0]];	
	cbw.bCBWCBLength = cdb_len;
	memcpy(cbw.CBWCB, cdb, cdb_len);
	printk(KERN_INFO "sending DATA read command\n");
	r1 = usb_bulk_msg(udev, usb_sndbulkpipe(udev,endpoint_out_addr),(unsigned char*)&cbw, 31, &size3, 1000000);
	   while(r1!=0 && ct<5)  //if not sent first time
	{
		r1=usb_clear_halt(udev,usb_sndbulkpipe(udev,endpoint_out_addr)) ;
		r1 = usb_bulk_msg(udev, usb_sndbulkpipe(udev,endpoint_out_addr),(unsigned char*)&cbw, 31, &size3, 1000);
		ct++;

	}
	if(ct==5 && r!=0)
	{	ct=0;
		return r;
	}
	printk(KERN_INFO "   sent %d CDB bytes and sent %d CBW bytes\n", cdb_len,size3);
	
	i=usb_bulk_msg(udev, usb_rcvbulkpipe (udev,endpoint_in_addr), (unsigned char*)buffer,  hi, &size2, 1000000);
	printk(KERN_INFO "   received %d bytes\n", size2);
	if(i!=0)
	{
		return i;
	}
	k=get_status();  //check status
	if(k==-1)
	{
		return -1;
	}

	printk(KERN_INFO "data is \n");
	//for (pooja=0;pooja<READ_DATA;pooja++)
	printk(KERN_INFO "%u ",*buffer);   //[0],buffer2[1],buffer2[2]);
	return 0;

}
//------------only write--------------------

int only_write(sector_t sector_off, u8 *buffer, unsigned int sectors)
{
	//u8 buffer3[512];
   unsigned int hello;
	int i,r,k,r1;			 //error codes for: bulk send , bulk recieve, stauts 
	int size4,size5;  //received sizes
	u8 cdb_len;          //CBWCB length
	//static u32 tag = 4;
	int ct=0;   //incase of stall 
	struct command_block_wrapper cbw;
   memset(&cbw, 0, sizeof(cbw));
	memset(cdb, 0, sizeof(cdb));
	//memset(buffer3, 0, sizeof(buffer3));
	r=0;
	cbw.dCBWSignature[0] = 'U';
	cbw.dCBWSignature[1] = 'S';
	cbw.dCBWSignature[2] = 'B';
	cbw.dCBWSignature[3] = 'C';	
	cbw.bCBWLUN = lun;
	cbw.bmCBWFlags = USB_DIR_OUT;	
	cdb[0] = 0x2a;
	cdb[1] = 0x00;
	cdb[2] = (sector_off & 0xFF000000) >> 24;
	cdb[3] = (sector_off & 0xFF0000) >> 16;
	cdb[4] = (sector_off & 0xFF00) >> 8;
	cdb[5] = (sector_off & 0xFF);
	cdb[7] = ((u16)sectors & 0xff00);
	cdb[8] = ((u16)sectors & 0x00ff);
	tag++;
	cbw.dCBWTag = tag;
	exp_tag=tag;
	hello=512*sectors;
	cbw.dCBWDataTransferLength = hello; //
	cdb_len = cdb_length[cdb[0]];	
	cbw.bCBWCBLength = cdb_len;
	memcpy(cbw.CBWCB, cdb, cdb_len);


   r1=usb_clear_halt(udev,usb_rcvbulkpipe(udev,endpoint_in_addr)) ;
	printk(KERN_INFO "f1 r1=%d\n",r1);
	r1=usb_clear_halt(udev,usb_sndbulkpipe(udev,endpoint_out_addr)) ;	
	printk(KERN_INFO "s1 r1=%d\n",r1);

	printk(KERN_INFO "sending DATA write command\n");
	r1 = usb_bulk_msg(udev, usb_sndbulkpipe(udev,endpoint_out_addr),(unsigned char*)&cbw, 31, &size4, 1000000);
	   while(r1!=0 && ct<5)  //if not sent first time
	{
		r1=usb_clear_halt(udev,usb_sndbulkpipe(udev,endpoint_out_addr)) ;
		r1 = usb_bulk_msg(udev, usb_sndbulkpipe(udev,endpoint_out_addr),(unsigned char*)&cbw, 31, &size4, 1000000);
		ct++;

	}
	if(ct==5 && r!=0)
	{	ct=0;
		return r;
	}
	printk(KERN_INFO "   sent %d CDB bytes and sent %d CBW bytes\n", cdb_len,size4);
	
	i=usb_bulk_msg(udev, usb_sndbulkpipe (udev,endpoint_out_addr), (unsigned char*)buffer,  hello, &size5, 1000000);
	printk(KERN_INFO "   sent %d bytes\n", size5);
	if(i!=0)
	{
		return i;
	}
	
	r1=usb_clear_halt(udev,usb_sndbulkpipe(udev,endpoint_out_addr)) ;
	printk(KERN_INFO "f r1=%d\n",r1);
	r1=usb_clear_halt(udev,usb_rcvbulkpipe(udev,endpoint_out_addr)) ;
	printk(KERN_INFO "s r1=%d\n",r1);
	k=get_status();

	return 0;



}

static int rb_open(struct block_device *bdev, fmode_t mode)
{
	unsigned unit = iminor(bdev->bd_inode);

	printk(KERN_INFO "poojas: Device is opened\n");
	printk(KERN_INFO "poojas: Inode number is %d\n", unit);

	if (unit > 2)
		return -ENODEV;
	return 0;
}

static int rb_close(struct gendisk *disk, fmode_t mode)
{
	printk(KERN_INFO "poojas: Device is closed\n");
	return 0;
}


//transfer


static int rb_transfer(struct work_struct *work)
{
	//struct rb_device *dev = (struct rb_device *)(req->rq_disk->private_data);
	struct my_work *mwp = container_of(work, struct my_work, work);
	struct request *req=mwp->data;
	int dir = rq_data_dir(req);
	sector_t start_sector = blk_rq_pos(req);
	unsigned int sector_cnt = blk_rq_sectors(req);

	struct bio_vec *bv;
	struct req_iterator iter;

	sector_t sector_offset;
	unsigned int sectors;
	u8 *buffer;

	int ret = 0;
	int er;
	printk(KERN_DEBUG "rb: Dir:%d; Sec:%lld; Cnt:%d\n", dir, start_sector, sector_cnt);

	sector_offset = 0;
	rq_for_each_segment(bv, req, iter)
	{
		buffer = page_address(bv->bv_page) + bv->bv_offset;
		if (bv->bv_len % 512 != 0)       
		{
			printk(KERN_ERR "poojas: Should never happen: "
				"bio size (%d) is not a multiple of RB_SECTOR_SIZE (%d).\n"
				"This may lead to data truncation.\n",
				bv->bv_len, 512);
			ret = -EIO;
		}
		sectors = bv->bv_len / 512;
		printk(KERN_DEBUG "poojas: Sector Offset: %lld; Buffer: %p; Length: %d sectors\n",
			sector_offset, buffer, sectors);
		if (dir == WRITE) /* Write to the device */
		{
			
			er=only_write(start_sector + sector_offset, buffer, sectors);
			printk(KERN_INFO "error inside write block %d\n",er);
		}
		else /* Read from the device */
		{
			er=only_read(start_sector + sector_offset, buffer, sectors);
			printk(KERN_INFO "error inside READ block %d\n",er);
		}
		sector_offset += sectors;
	}
	if (sector_offset != sector_cnt)
	{
		printk(KERN_ERR "poojas: bio info doesn't match with the request info");
		ret = -EIO;
	}
	__blk_end_request_all(req, ret);
	return;
}

//rb request

static void rb_request(struct request_queue *q)
{
	struct request *req;
	int ret;
	struct my_work *working=NULL;
	/* Gets the current request from the dispatch queue */

	while ((req = blk_fetch_request(q)) != NULL)
	{
#if 0
		/*
		 * This function tells us whether we are looking at a filesystem request
		 * - one that moves block of data
		 */
		if (!blk_fs_request(req))
		{
			printk(KERN_NOTICE "poojas: Skip non-fs request\n");
			/* We pass 0 to indicate that we successfully completed the request */
			__blk_end_request_all(req, 0);
			continue;
		}
#endif
			working=(struct my_work*)kmalloc(sizeof (struct my_work),GFP_ATOMIC) ;
			if(working==NULL)
						{

					__blk_end_request_all(req, 0);
						printk(KERN_INFO "no memo alocation\n");
						break;

						}
			working->data = req;
			INIT_WORK(&working->work, rb_transfer);
			queue_work(myqueue, &working->work);
		//ret = rb_transfer(req);
		
	}
	return;
}






static struct block_device_operations rb_fops =
{
	.owner = THIS_MODULE,
	.open = rb_open,
	.release = rb_close,
};


//________________initilize as block devie______________________//

int initblock()
{
	  //maximum logical block adress and block size
	rb_dev.size =max_lba+1; //---------------------see

	/* Get Registered */
	rb_major = register_blkdev(rb_major, "poojas");
	if (rb_major <= 0)
	{
		printk(KERN_ERR "rb: Unable to get Major Number\n");
		return -EBUSY;
	}

	/* Get a request queue (here queue is created) */
	spin_lock_init(&rb_dev.lock);
	rb_dev.rb_queue = blk_init_queue(rb_request, &rb_dev.lock);
	if (rb_dev.rb_queue == NULL)
	{
		printk(KERN_ERR "rb: blk_init_queue failure\n");
		unregister_blkdev(rb_major, "poojas");
		return -ENOMEM;
	}
	
	/*
	 * Add the gendisk structure
	 * By using this memory allocation is involved, 
	 * the minor number we need to pass bcz the device 
	 * will support this much partitions 
	 */
	rb_dev.rb_disk = alloc_disk(2);                     ////////////////see
	if (!rb_dev.rb_disk)
	{
		printk(KERN_ERR "poojas: alloc_disk failure\n");
		blk_cleanup_queue(rb_dev.rb_queue);
		unregister_blkdev(rb_major, "poojas");
		return -ENOMEM;
	}
	myqueue = create_workqueue("myqueue");
	//INIT_WORK(&my_work.work, rb_transfer);

 	/* Setting the major number */
	rb_dev.rb_disk->major = rb_major;
  	/* Setting the first mior number */
	rb_dev.rb_disk->first_minor = 0;
 	/* Initializing the device operations */
	rb_dev.rb_disk->fops = &rb_fops;
 	/* Driver-specific own internal data */
	rb_dev.rb_disk->private_data = &rb_dev;
	rb_dev.rb_disk->queue = rb_dev.rb_queue;
	/*
	 * You do not want partition information to show up in 
	 * cat /proc/partitions set this flags
	 */
	//rb_dev.rb_disk->flags = GENHD_FL_SUPPRESS_PARTITION_INFO;
	sprintf(rb_dev.rb_disk->disk_name, "poojas");
	/* Setting the capacity of the device in its gendisk structure */
	set_capacity(rb_dev.rb_disk, rb_dev.size);

	/* Adding the disk to the system */
	add_disk(rb_dev.rb_disk);
	/* Now the disk is "live" */
	printk(KERN_INFO "poojas: Ram Block driver initialised (%d sectors; %d bytes)\n",
		rb_dev.size, rb_dev.size * block_size1);

	return 0;




}



static void usbdev_disconnect(struct usb_interface *interface)
{
	printk(KERN_INFO "USBDEV Device Removed\n");
	flush_workqueue(myqueue);
	destroy_workqueue(myqueue);
	del_gendisk(rb_dev.rb_disk);
	put_disk(rb_dev.rb_disk);
	blk_cleanup_queue(rb_dev.rb_queue);
	unregister_blkdev(rb_major, "poojas");
	//spin_unlock(&rb_dev.lock);        ------------------see
	return;
}


static int usbdev_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	int i;
	int err1;
	u8 epAddr, epAttr;  //for end point adress and end point attribute
	struct usb_endpoint_descriptor *ep_desc;
	udev = interface_to_usbdev(interface);
	
	if(id->idProduct == CARD_READER_PID)
	{
		printk(KERN_INFO "Known USB drivedetected\n");
		printk(KERN_INFO "card reader plugged in\n");
	}
	else if(id->idProduct == SONY_PID)
	{
		printk(KERN_INFO "Known USB drivedetected\n");
		printk(KERN_INFO "sony plugged in\n");
	}
	else if(id->idVendor == SANDISK_PID)
	{
		printk(KERN_INFO "------Known USB drivedetected-------\n");
		printk(KERN_INFO "sandisk Plugged in\n");
	}

	printk(KERN_INFO "USB DEVICE VID : %x\n", id->idVendor);
	printk(KERN_INFO "USB DEVICE PID : %x\n", id->idProduct);
	printk(KERN_INFO "USB DEVICE CLASS : %x\n", interface->cur_altsetting->desc.bInterfaceClass);
	printk(KERN_INFO "USB DEVICE SUB CLASS : %x\n", interface->cur_altsetting->desc.bInterfaceSubClass);
	printk(KERN_INFO "USB DEVICE Protocol : %x\n", interface->cur_altsetting->desc.bInterfaceProtocol);
	printk(KERN_INFO "No. of Endpoints = %d\n", interface->cur_altsetting->desc.bNumEndpoints);
	printk(KERN_INFO "No. of Altsettings = %d\n",interface->num_altsetting);

	for(i=0;i<interface->cur_altsetting->desc.bNumEndpoints;i++)
	{
		ep_desc = &interface->cur_altsetting->endpoint[i].desc;
		epAddr = ep_desc->bEndpointAddress;
		epAttr = ep_desc->bmAttributes;
	
		if((epAttr & USB_ENDPOINT_XFERTYPE_MASK)==USB_ENDPOINT_XFER_BULK)
		{
			if(epAddr & USB_DIR_IN){
				endpoint_in_addr=epAddr;
				printk(KERN_INFO "EP %d is Bulk IN address=%x \n", i,endpoint_in_addr); 
				
		
				}
			else{
				endpoint_out_addr=epAddr;
				printk(KERN_INFO "EP %d is Bulk OUT address=%x\n", i, endpoint_out_addr);
				
				}
	
		}

	}

	
	printk(KERN_INFO "resetting device\n");
	reset_dev();
	printk(KERN_INFO "READING capacity:\n");
	err1=find_lun();
	if(err1!=0)
	{
		 return err1;
	}
	err1=only_inq();	
	if(err1!=0)
	{
		 return err1;
	}
	err1=only_cap();	
	if(err1!=0)
	{
		 return err1;
	}

	err1=initblock();
	if(err1!=0)
	{
		 return err1;
	}
	return 0;
}



/*Operations structure*/
static struct usb_driver usbdev_driver = {
	name: "usbdev",  //name of the device
	probe: usbdev_probe, // Whenever Device is plugged in
	disconnect: usbdev_disconnect, // When we remove a device
	id_table: usbdev_table, //  List of devices served by this driver
};


int device_init(void)
{	
	int res;
	
	res=usb_register(&usbdev_driver);
	if(res!=0)
	{
		printk("usb_register failed. Error number %d\n", res);
		return res;
	}
	printk(KERN_INFO "--------UAS READ Capacity Driver Inserted------\n");
	return 0;
}

void device_exit(void)
{
	usb_deregister(&usbdev_driver);
	printk(KERN_NOTICE "-----Leaving Kernel----\n");

	return 0;
}

module_init(device_init);
module_exit(device_exit);
MODULE_LICENSE("GPL");

