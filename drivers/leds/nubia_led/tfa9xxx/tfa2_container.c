/*
 * Copyright 2014-2018 NXP Semiconductors
 * NOT A CONTRIBUTION.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "tfa2_dev.h" /* for regwrite */
#include "tfa2_container.h"

/*
 dsp mem direct write
 */
#if 0 //TODO optimize to burst
static int tfa2_cnt_write_dspmem(struct tfa2_device *tfa, nxpTfaDspMem_t *cfmem)
{
	int error;
	int i;

	if (tfa->verbose)
		dev_dbg(&tfa->i2c->dev, "writing dsp mem (%d): 0x%02x %d words\n", cfmem->type, cfmem->address, cfmem->size);

	return tfa2_i2c_write_cf_mem(tfa->i2c, cfmem->address,)
;
}
#endif
static int tfa2_cnt_write_dspmem(struct tfa2_device *tfa, nxpTfaDspMem_t *cfmem)
{
	int error = 0;
	int i;

	for(i=0;i<cfmem->size;i++) {
		if (tfa->verbose)
			dev_dbg(&tfa->i2c->dev, "dsp mem (%d): 0x%02x=0x%04x\n", cfmem->type, cfmem->address, cfmem->words[i]);
		error = tfa2_i2c_write_cf_mem32(tfa->i2c, cfmem->address++, &cfmem->words[i], 1, cfmem->type);
		if (error)
			return error;
	}

	return error;
}
/*
 * check the container file
*/
int tfa2_load_cnt(void *cnt, int length)
{
	nxpTfaContainer_t  *cntbuf = (nxpTfaContainer_t  *)cnt;

	if (length > TFA_MAX_CNT_LENGTH) {
		pr_err("incorrect length\n");
		return -EINVAL;
	}

	if (HDR(cntbuf->id[0],cntbuf->id[1]) == 0) {
		pr_err("header is 0\n");
		return -EINVAL;
	}

	if ( (HDR(cntbuf->id[0],cntbuf->id[1])) != paramsHdr ) {
		pr_err("wrong header type: 0x%02x 0x%02x\n", cntbuf->id[0],cntbuf->id[1]);
		return -EINVAL;
	}

	if (cntbuf->size == 0) {
		pr_err("data size is 0\n");
		return -EINVAL;
	}

	/* check CRC */
	if ( tfa2_cnt_crc_check_container(cntbuf)) {
		pr_err("CRC error\n");
		return -EINVAL;
	}

	/* check sub version level */
	if ((cntbuf->subversion[1] != NXPTFA_PM_SUBVERSION) &&
							(cntbuf->subversion[0] != '0')) {
		pr_err("container sub-version not supported: %c%c\n",
				cntbuf->subversion[0], cntbuf->subversion[1]);
		return -EINVAL;
	}

	return 0;
}

/*
 * check CRC for container
 *   CRC is calculated over the bytes following the CRC field
 *
 *   return non zero value on error
 */
int tfa2_cnt_crc_check_container(nxpTfaContainer_t *cont)
{
	uint8_t *base;
	size_t size;
	uint32_t crc;

	base = (uint8_t *)&cont->CRC + 4; // ptr to bytes following the CRC field
	size = (size_t)(cont->size - (base - (uint8_t *)cont)); // nr of bytes following the CRC field
	crc = ~crc32_le(~0u, base, size);

	return crc != cont->CRC;
}

/*****************************************************************************/
/*   cnt getinfo  */


char *tfa2_cnt_get_string(nxpTfaContainer_t *cnt, nxpTfaDescPtr_t *dsc)
{
	if ( dsc->type != dscString)
		return "Undefined string"; // TODO: why not return NULL to indicate error?

	return dsc->offset+(char*)cnt;
}

/*
 * Get the name of the device at a certain index in the container file
 *  return device name
 */
char  *tfa2_cnt_device_name(nxpTfaContainer_t *cnt, int dev_idx)
{
	nxpTfaDeviceList_t *dev;

	dev = tfa2_cnt_device(cnt, dev_idx);
	if (dev == NULL)
		return "!ERROR!"; // TODO: why not return NULL to indicate error?

	return tfa2_cnt_get_string(cnt, &dev->name);
}
/*
 * Get the name of the profile at certain index for a device in the container file
 *  return profile name
 */
char *tfa2_cnt_profile_name(nxpTfaContainer_t *cnt, int dev_idx, int prof_idx)
{
	nxpTfaProfileList_t *prof = NULL;
	char *str;

	/* the Nth profiles for this device */
	prof = tfa2_cnt_get_dev_prof_list(cnt, dev_idx, prof_idx);

	/* If the index is out of bound */
	if (prof == NULL)
		return "NONE"; // TODO: why not return NULL to indicate error?

	str =  tfa2_cnt_get_string(cnt, &prof->name);
	return str;
}
/*
 * Get the application name from the container file application field
 * note that the input stringbuffer should be sizeof(application field)+1
 *
 */
int tfa2_cnt_get_app_name(struct tfa2_device *tfa, char *name)
{
	unsigned int i;
	int len = 0;

	for(i=0; i<sizeof(tfa->cnt->application); i++) {
		if (isalnum(tfa->cnt->application[i])) /* copy char if valid */
			name[len++] = tfa->cnt->application[i];
		if (tfa->cnt->application[i]=='\0')
			break;
	}
	name[len++] = '\0';

	return len;
}


/*
 * Dump the contents of the file header
 */
void tfa2_cnt_show_header(nxpTfaHeader_t *hdr) {
	char _id[2];

	pr_debug("File header\n");

	_id[1] = hdr->id >> 8;
	_id[0] = hdr->id & 0xff;
	pr_debug("\tid:%.2s version:%.2s subversion:%.2s\n", _id,
		   hdr->version, hdr->subversion);
	pr_debug("\tsize:%d CRC:0x%08x \n", hdr->size, hdr->CRC);
	pr_debug( "\tcustomer:%.8s application:%.8s type:%.8s\n", hdr->customer,
			   hdr->application, hdr->type); /* TODO fix leading zeroes */
}


/*****************************************************************************/
/*   cnt infra lookups  */


/*
 * get the slave for the device if it exists
 */
int tfa2_cnt_get_slave(nxpTfaContainer_t *cnt, int dev_idx)
{
	nxpTfaDeviceList_t *dev;

	/* Make sure the cnt file is loaded */
	if (!cnt)
		return -EINVAL;

	dev = tfa2_cnt_device(cnt, dev_idx);
	if (!dev)
		return -EINVAL;

	return dev->dev;
}

/*
 * return the device list pointer
 */
nxpTfaDeviceList_t *tfa2_cnt_device(nxpTfaContainer_t *cnt, int dev_idx)
{
	return tfa2_cnt_get_dev_list(cnt, dev_idx);
}

/*
 * return device list dsc from index
 */
nxpTfaDeviceList_t *tfa2_cnt_get_dev_list(nxpTfaContainer_t *cont, int dev_idx)
{
	uint8_t *base = (uint8_t *) cont;
	nxpTfaDeviceList_t *this_dev;

	if (cont == NULL)
		return NULL;

	if ( (dev_idx < 0) || (dev_idx >= cont->ndev))
		return NULL;

	if (cont->index[dev_idx].type != dscDevice)
		return NULL;

	base += cont->index[dev_idx].offset;
	this_dev = (nxpTfaDeviceList_t *) base;

	if ( this_dev->name.type != dscString) {
		pr_err("fatal corruption: device[%d] has no name\n", dev_idx);
		return NULL;
	}

	return this_dev;
}

/*
 * get the Nth profile for the Nth device
 */
nxpTfaProfileList_t *tfa2_cnt_get_dev_prof_list(nxpTfaContainer_t * cont, int devIdx, int profIdx)
{
	nxpTfaDeviceList_t *dev;
	int idx, hit;
	uint8_t *base = (uint8_t *) cont;

	dev = tfa2_cnt_get_dev_list(cont, devIdx);
	if (dev) {
		for (idx = 0, hit = 0; idx < dev->length; idx++) {
			if (dev->list[idx].type == dscProfile) {
				if (profIdx == hit++)
					return (nxpTfaProfileList_t *) (dev->list[idx].offset+base);
			}
		}
	}

	return NULL;
}

/*
 * get the number of profiles for the Nth device of this tfa
 */
int tfa2_dev_get_dev_nprof(struct tfa2_device *tfa) //TODO call tfa2_cnt_get_dev_nprof
{
	nxpTfaDeviceList_t *dev;
	int idx, nprof = 0;

	if (tfa->cnt == NULL)
		return 0;

	if ((tfa->dev_idx < 0) || (tfa->dev_idx >= tfa->cnt->ndev))
		return 0;

	dev = tfa2_cnt_get_dev_list(tfa->cnt, tfa->dev_idx);
	if (dev) {
		for (idx = 0; idx < dev->length ; idx++) {
			if (dev->list[idx].type == dscProfile) {
				nprof++;
			}
		}
	}

	return nprof;
}

/*
 * get the number of profiles for the Nth device
 */
int tfa2_cnt_get_dev_nprof( nxpTfaContainer_t * cnt, int dev_idx)
{
	nxpTfaDeviceList_t *dev;
	int idx, nprof = 0;

	if (cnt == NULL)
		return 0;

	if ((dev_idx < 0) || (dev_idx >= cnt->ndev))
		return 0;

	dev = tfa2_cnt_get_dev_list(cnt, dev_idx);
	if (dev) {
		for (idx = 0; idx < dev->length; idx++) {
			if (dev->list[idx].type == dscProfile) {
				nprof++;
			}
		}
	}

	return nprof;
}


/*
 * get the 1st profilename match for this  device
 */
int tfa2_cnt_grep_profile_names(nxpTfaContainer_t * cnt, int dev_idx, const char *string)
{
	int prof;
	//int maxprof = tfa2_cnt_get_dev_nprof(cnt, dev_idx);

	/* compare srting to  the  profile name  in the list of profiles */
	for (prof = 0; prof < cnt->nprof; prof++) {
		if(strstr(tfa2_cnt_profile_name(cnt, dev_idx, prof), string) != 0) {
			return prof; //tfa2_cnt_get_dev_prof_list(cnt, dev_idx, prof);
		}
	}

	return -EINVAL;
}

/*
 * write the register based on the input address, value and mask
 *  only the part that is masked will be updated
 */
static int tfa2_dev_cnt_write_register(struct tfa2_device *tfa, nxpTfaRegpatch_t *reg)
{
	int rc;
	uint16_t value,newvalue;

	if (tfa->verbose)
		dev_dbg(&tfa->i2c->dev, "register: 0x%02x=0x%04x (msk=0x%04x)\n", reg->address, reg->value, reg->mask);

	rc = tfa2_i2c_read_reg(tfa->i2c, reg->address); /* will report error */
	if (rc < 0)
		return rc;
	value = (uint16_t)rc;

	value &= ~reg->mask;
	newvalue = reg->value & reg->mask;

	value |= newvalue;

	rc = tfa2_i2c_write_reg(tfa->i2c,reg->address, value);

	return rc ;

}

/* write reg and bitfield items in the devicelist to the target */
int tfa2_cnt_write_regs_dev(struct tfa2_device *tfa)
{
	nxpTfaDeviceList_t *dev = tfa2_cnt_device(tfa->cnt, tfa->dev_idx);
	nxpTfaBitfield_t *bitf;
	int i, err = 0;

	if ( !dev ) {
		return -EINVAL;
	}

	/* process the list until a patch, file of profile is encountered */
	for(i=0;i<dev->length ;i++) {
		if ( dev->list[i].type == dscPatch ||
			  dev->list[i].type ==dscFile  ||
			  dev->list[i].type ==dscProfile ) break;

		if  ( dev->list[i].type == dscBitfield) {
			bitf = (nxpTfaBitfield_t *)( dev->list[i].offset+(uint8_t *)tfa->cnt);
			err = tfa2_dev_write_bitfield_volatile(tfa, bitf);
		}
		if  ( dev->list[i].type == dscRegister ) {
			err = tfa2_dev_cnt_write_register(tfa, (nxpTfaRegpatch_t *)( dev->list[i].offset+(char*)tfa->cnt));
		}

		if ( err )
			break;
	}

	return err;
}

/* write reg and bitfield items in the profilelist the target */
int tfa2_cnt_write_regs_profile(struct tfa2_device *tfa, int prof_idx)
{
	nxpTfaProfileList_t *prof = tfa2_cnt_get_dev_prof_list(tfa->cnt, tfa->dev_idx, prof_idx);
	nxpTfaBitfield_t *bitf;
	unsigned int i;
	int err = 0;

	if ( !prof ) {
		return -EINVAL;
	}

	if (tfa->verbose)
		dev_dbg(&tfa->i2c->dev, "----- profile: %s (%d) -----\n", tfa2_cnt_get_string(tfa->cnt, &prof->name), prof_idx);

	/* process the list until the end of the profile or the default section */
	for(i=0;i<prof->length-1;i++) {
		/* We only want to write the values before the default section when we switch profile */
		if(prof->list[i].type == dscDefault)
			break;

		if  ( prof->list[i].type == dscBitfield) {
			bitf = (nxpTfaBitfield_t *)( prof->list[i].offset+(uint8_t *)tfa->cnt);
			err = tfa2_dev_write_bitfield_volatile(tfa, bitf);
		}
		if  ( prof->list[i].type == dscRegister ) {
			err = tfa2_dev_cnt_write_register(tfa, (nxpTfaRegpatch_t *)( prof->list[i].offset+(char*)tfa->cnt));
		}
		if ( err ) break;
	}
	return err;
}

static int tfa2_cnt_write_patch(struct tfa2_device *tfa, nxpTfaPatch_t *patchfile)
{
	int rc , size;

		if (tfa->verbose)
			tfa2_cnt_show_header(&patchfile->hdr);

		size = patchfile->hdr.size - sizeof(nxpTfaPatch_t ); // size is total length
		rc = tfa2_check_patch((const uint8_t *)patchfile,  patchfile->hdr.size, tfa->rev ); //TODO fix for single patch header type
		if ( rc < 0 )
			return rc;

		rc = tfa2_dev_dsp_patch(tfa, size, (const unsigned char *) patchfile->data);

		return rc;
}
static int tfa2_cnt_write_item(struct tfa2_device *tfa, nxpTfaDescPtr_t * dsc)
{
	int rc = 0;
	nxpTfaRegpatch_t *reg;
	nxpTfaBitfield_t *bitf;

	switch (dsc->type) {
        case dscDefault:
	case dscDevice: // ignore
	case dscProfile:    // profile list
		break;
	case dscRegister:   // register patch
		reg = (nxpTfaRegpatch_t *)(dsc->offset+(uint8_t *)tfa->cnt);
		return tfa2_dev_cnt_write_register(tfa, reg);
		//dev_dbg(&tfa->i2c->dev, "$0x%2x=0x%02x,0x%02x\n", reg->address, reg->mask, reg->value);
		break;
	case dscString: // ascii: zero terminated string
		dev_dbg(&tfa->i2c->dev, ";string: %s\n", tfa2_cnt_get_string(tfa->cnt, dsc));
		break;
	case dscFile:       // filename + file contents
	case dscPatch:
		rc = tfa2_cnt_write_file(tfa, (nxpTfaFileDsc_t *)dsc);
		break;
	case dscCfMem:
		rc = tfa2_cnt_write_dspmem(tfa, (nxpTfaDspMem_t *)(dsc->offset+(uint8_t *)tfa->cnt));
		break;
	case dscBitfield:
		bitf = (nxpTfaBitfield_t *)(dsc->offset+(uint8_t *)tfa->cnt);
		rc = tfa2_i2c_write_bf_volatile(tfa->i2c , bitf->field, bitf->value);
		break;
	default:
		dev_err(&tfa->i2c->dev, "unsupported list item:%d\n", dsc->type);
		rc = -EINVAL;
//	case dscFilter:
//		return tfaRunWriteFilter(tfa, (nxpTfaContBiquad_t *)(dsc->offset+(uint8_t *)tfa->cnt));
//		break;
	}

	return rc;
}

/*
 * all container originated RPC msgs goes through this
 *
 *  the cmd_id is monitored here to insert 		{"SetAlgoParams", 0x48100},
		{"SetAlgoParamsWithoutReset", 0x48102},
		{"SetMBDrc", 0x48107},
		{"SetMBDrcWithoutReset", 0x48108},
 */
int tfa2_cnt_write_msg(struct tfa2_device *tfa , int wlength, char *wbuf) {
	int rc  = 0 ;
	uint8_t *cmd_id24 = (uint8_t *)wbuf;
	int cmd_id = cmd_id24[0] <<16 |  cmd_id24[1] << 8 | cmd_id24[2];
	uint8_t *cmd_lsb =  &cmd_id24[2];
	int coldstarting = tfa2_dev_is_fw_cold(tfa); /* set if FW is cold */

	/*
	 * select the cmd_id variant based on the init state
	 */
	switch ( cmd_id & 0x7fff ) {
	default:
		break;
	case 0x100:  /* SetAlgoParams 0x48100*/
	case 0x102:  /* SetAlgoParamsWithoutReset 0x48102*/
		*cmd_lsb = coldstarting ? 0  : 2; /* if cold cmd_id = coldstart variant */
		break;
	case 0x107:  /* SetMBDrc 0x48107*/
	case 0x108:  /* SetMBDrcWithoutReset 0x48108*/
		*cmd_lsb = coldstarting ? 7 : 8; /* if cold cmd_id = coldstart variant */
		break;
	}

	if (tfa->verbose) {
		dev_dbg(&tfa->i2c->dev, "Writing cmd=0x%06x,size=%d\n", cmd_id24[0] <<16 |  cmd_id24[1] << 8 | cmd_id24[2], wlength);
	}
	rc =  tfa2_i2c_dsp_execute( tfa->i2c, wbuf, wlength, NULL, 0 );

	return rc;
}

/*  write  all patchfiles in the devicelist to the target */
int tfa2_cnt_write_patches(struct tfa2_device *tfa) {
	int rc  = 0 ;
	nxpTfaDeviceList_t *dev = tfa2_cnt_get_dev_list(tfa->cnt, tfa->dev_idx);
	nxpTfaFileDsc_t *file;
	nxpTfaPatch_t *patchfile;
	int i;

	if ( !dev ) {
		return -EINVAL;
	}
	/* process the list until a patch  is encountered and load it */
	for(i=0;i<dev->length ;i++) {
		if ( dev->list[i].type == dscPatch ) {
			file = (nxpTfaFileDsc_t *)(dev->list[i].offset+(uint8_t *)tfa->cnt);
			patchfile =(nxpTfaPatch_t *)&file->data;
			rc = tfa2_cnt_write_patch(tfa,patchfile);
		}
	}
	return rc;
}
/* write all param files in the devicelist to the target */
int tfa2_cnt_write_files(struct tfa2_device *tfa)
{
	nxpTfaDeviceList_t *dev;
	nxpTfaFileDsc_t *file;
	int err = 0;
	//char buffer[(MEMTRACK_MAX_WORDS * 3) + 3] = {0}; //every word requires 3 and 3 is the msg
	int i, size = 0;

	dev = tfa2_cnt_device(tfa->cnt, tfa->dev_idx);
	if (!dev)
		return -EINVAL;

	/* process the list and write all files  */
	for(i=0;i < dev->length  ;i++) {
		switch (dev->list[i].type) {
		case dscFile:
			file = (nxpTfaFileDsc_t *)(dev->list[i].offset+(uint8_t *)tfa->cnt);
			err = tfa2_cnt_write_file(tfa, file);
			break;
		case dscCmd:
			size = *(uint16_t *)(dev->list[i].offset+(char*)tfa->cnt);
			err = tfa2_cnt_write_msg(tfa, size, dev->list[i].offset+2+(char*)tfa->cnt);
			break;
		case dscCfMem:
			err = tfa2_cnt_write_dspmem(tfa, (nxpTfaDspMem_t *)(dev->list[i].offset+(uint8_t *)tfa->cnt));
			break;
		default:
			/* ignore others */
			break;
		}
	}

	return err;
}
/* write all param files in the profile list  to the target */
int tfa2_cnt_write_files_profile(struct tfa2_device *tfa, int prof_idx, int vstep_idx)
{
	nxpTfaProfileList_t *prof = tfa2_cnt_get_dev_prof_list(tfa->cnt, tfa->dev_idx, prof_idx);
	nxpTfaFileDsc_t *file;
	int rc = 0;
	//char buffer[(MEMTRACK_MAX_WORDS * 3) + 3] = {0}; //every word requires 3 and 3 is the msg
	int i, size = 0;

	if ( !prof ) {
		return -EINVAL;
	}
	/* process the list and write all files  */
	for(i=0; i < prof->length-1; i++) {
		switch (prof->list[i].type) {
		case dscFile:
			file = (nxpTfaFileDsc_t *)(prof->list[i].offset+(uint8_t *)tfa->cnt);
			rc = tfa2_cnt_write_file(tfa, file);
			break;
		case dscCmd:
			size = *(uint16_t *)(prof->list[i].offset+(char*)tfa->cnt);
			rc = tfa2_cnt_write_msg(tfa, size, prof->list[i].offset+2+(char*)tfa->cnt);
			break;
		case dscCfMem:
			rc = tfa2_cnt_write_dspmem(tfa, (nxpTfaDspMem_t *)(prof->list[i].offset+(uint8_t *)tfa->cnt));
			break;
		case dscPatch:
			file = (nxpTfaFileDsc_t *)(prof->list[i].offset+(uint8_t *)tfa->cnt);
			rc = tfa2_cnt_write_patch(tfa,(nxpTfaPatch_t *)&file->data); /* data is the patch header */
			break;
		case dscDefault: /* the default label must not be passed */
			return 0;
			break;
		default:
			/* ignore others */
			break;
		}
	}

	return rc;
}

/*
 * write a parameter file to the device

 */
int tfa2_cnt_write_file(struct tfa2_device *tfa,  nxpTfaFileDsc_t *file) //TODO files support
{
	int err = 0;
	nxpTfaHeader_t *hdr = (nxpTfaHeader_t *)file->data;
	nxpTfaHeaderType_t type;
	int size;

	if (tfa->verbose) {
		tfa2_cnt_show_header(hdr);
	}

	type = (nxpTfaHeaderType_t) hdr->id;

	switch (type) {
	case msgHdr: /* generic DSP message */
		size = hdr->size - sizeof(nxpTfaMsgFile_t);
		err = tfa2_cnt_write_msg(tfa, size, (char *)((nxpTfaMsgFile_t *)hdr)->data);
		break;
//	case volstepHdr:
//		if (tfa->tfa2_family == 2) {
//			err = tfa2_cnt_write_vstepMax2(tfa, (nxpTfaVolumeStepMax2File_t *)hdr, vstep_idx, vstep_msg_idx);
//		} else {
//			err = tfa2_cnt_write_vstep(tfa, (nxpTfaVolumeStep2File_t *)hdr, vstep_idx);
//		}
//		break;
	case speakerHdr:
			/* Remove header and xml_id */
			size = hdr->size - sizeof(struct nxpTfaSpkHeader) - sizeof(struct nxpTfaFWVer);
			err = tfa2_cnt_write_msg(tfa, size,
					(char *)(((nxpTfaSpeakerFile_t *)hdr)->data + (sizeof(struct nxpTfaFWVer))));

		break;
//	case presetHdr:
//		size = hdr->size - sizeof(nxpTfaPreset_t);
//		err = tfa98xx_dsp_write_preset(tfa, size, (const unsigned char *)((nxpTfaPreset_t *)hdr)->data);
//		break;
//	case equalizerHdr:
//		err = tfa2_cnt_write_filterbank(tfa, ((nxpTfaEqualizerFile_t *)hdr)->filter);
//		break;
//	case patchHdr:
//		size = hdr->size - sizeof(nxpTfaPatch_t ); // size is total length
//		err = tfa2_dsp_patch(tfa,  size, (const unsigned char *) ((nxpTfaPatch_t *)hdr)->data);
//		break;
//	case drcHdr:
//		if(hdr->version[0] == NXPTFA_DR3_VERSION) {
//			/* Size is total size - hdrsize(36) - xmlversion(3) */
//			size = hdr->size - sizeof(nxpTfaDrc2_t);
//			err = tfa2_cnt_write_drc_file(tfa, size, ((nxpTfaDrc2_t *)hdr)->data);
//		} else {
//			/*
//			 * The DRC file is split as:
//			 * 36 bytes for generic header (customer, application, and type)
//			 * 127x3 (381) bytes first block contains the device and sample rate
//			 * 				independent settings
//			 * 127x3 (381) bytes block the device and sample rate specific values.
//			 * The second block can always be recalculated from the first block,
//			 * if vlsCal and the sample rate are known.
//			 */
//			//size = hdr->size - sizeof(nxpTfaDrc_t);
//			size = 381; /* fixed size for first block */
//
//			//+381 is done to only send the second part of the drc block
//			err = tfa98xx_dsp_write_drc(tfa, size, ((const unsigned char *)((nxpTfaDrc_t *)hdr)->data+381));
//		}
//		break;
	case infoHdr:
		/* Ignore */
		break;
	default:
		dev_err(&tfa->i2c->dev, "Header is of unknown type: 0x%x\n", type);
		return -EINVAL;
	}

	return err;
}


/*
 *  process all items in the profilelist
 *   NOTE an error return during processing will leave the device muted
 *
 */
int tfa2_cnt_write_profile(struct tfa2_device *tfa, int prof_idx, int vstep_idx)
{
	int rc = 0;
	nxpTfaProfileList_t *prof = tfa2_cnt_get_dev_prof_list(tfa->cnt, tfa->dev_idx, prof_idx);
	nxpTfaProfileList_t *previous_prof = tfa2_cnt_get_dev_prof_list(tfa->cnt, tfa->dev_idx, tfa2_dev_get_swprofile(tfa));
	//char buffer[(MEMTRACK_MAX_WORDS * 4) + 4] = {0}; //every word requires 3 or 4 bytes, and 3 or 4 is the msg
	unsigned int i, j=0; // k0, tries=0;
	nxpTfaFileDsc_t *file;
	int size = 0; // ready, fs_previous_profile = 8; /* default fs is 48kHz*/

	if ( !prof || !previous_prof ) {
		dev_err(&tfa->i2c->dev, "Error trying to get the (previous) swprofile \n");
		return -EINVAL;
	}

	if (tfa->verbose) {
		dev_dbg(&tfa->i2c->dev, "device:%s profile:%s vstep:%d\n", tfa2_cnt_device_name(tfa->cnt, tfa->dev_idx),
					tfa2_cnt_profile_name(tfa->cnt, tfa->dev_idx,prof_idx),vstep_idx);
	}

	/* We only make a power cycle when the profiles are not in the same group */
	if (prof->group == previous_prof->group && prof->group != 0) {
		if (tfa->verbose) {
			dev_dbg(&tfa->i2c->dev, "The new profile (%s) is in the same group as the current profile (%s) \n",
				tfa2_cnt_get_string(tfa->cnt, &prof->name), tfa2_cnt_get_string(tfa->cnt, &previous_prof->name));
		}
	} else {
		/* mute */
		rc = tfa2_dev_mute(tfa, 1);
		if ( rc < 0 )
			return rc;

		/* When we switch profile we first power down the subsystem
		 * This should only be done when we are in operating mode
		 */
		rc = tfa2_dev_set_state(tfa, TFA_STATE_POWERDOWN);
		if ( rc < 0 )
			return rc;

	}

	/* set all bitfield settings */
	/* First set all default settings */
	if (tfa->verbose) {
		dev_dbg(&tfa->i2c->dev, "---------- default settings profile: %s (%d) ---------- \n",
				tfa2_cnt_get_string(tfa->cnt, &previous_prof->name), tfa2_dev_get_swprofile(tfa));
	}

	if( tfa->verbose)
		tfa2_show_current_state(tfa);

	/* Loop profile length */
	for(i=0;i<previous_prof->length-1;i++) {
		/* Search for the default section */
		if(i == 0) {
			while(previous_prof->list[i].type != dscDefault && i < previous_prof->length-1) {
				i++;
			}
			i++;
		}

		/* Only if we found the default section try writing the items */
		if(i < previous_prof->length-1) {
			rc =  tfa2_cnt_write_item(tfa,  &previous_prof->list[i]);
			if ( rc < 0 )
				return rc;
		}
	}

	if (tfa->verbose)
		dev_dbg(&tfa->i2c->dev, "---------- new settings profile: %s (%d) ---------- \n",
				tfa2_cnt_get_string(tfa->cnt, &prof->name), prof_idx);

	/* set new settings */
	for(i=0;i<prof->length-1;i++) {
		/* Remember where we currently are with writing items*/
		j = i;

		/* We only want to write the values before the default section when we switch profile */
		/* process and write all non-file items */
		switch (prof->list[i].type) {
			case dscFile:
			case dscPatch:
			case dscCmd:
			case dscDefault:
				/* When one of these files are found, we exit */
				i = prof->length-1;
				break;
			default:
				rc = tfa2_cnt_write_item(tfa, &prof->list[i]);
				if ( rc < 0 )
					return rc;
				break;
		}
	}
	/*
	 * not in any or same group
	 */
	if (prof->group != previous_prof->group || prof->group == 0) {
		/* Leave powerdown state */
		rc = tfa2_dev_set_state(tfa, TFA_STATE_POWERUP);
		if ( rc < 0 )
			return rc;

		//show_current_state(tfa);

	}

		/* write everything until end or the default section starts
		 * Start where we currenly left */
		for(i=j;i<prof->length-1;i++) {
			/* We only want to write the values before the default section when we switch profile */

			if(prof->list[i].type == dscDefault) {
					break;
			}

		switch (prof->list[i].type) {
			case dscFile:
			case dscPatch:
				file = (nxpTfaFileDsc_t *)(prof->list[i].offset+(uint8_t *)tfa->cnt);
				rc = tfa2_cnt_write_file(tfa, file);
				break;
			case dscCmd:
					size = *(uint16_t *)(prof->list[i].offset+(char*)tfa->cnt);
					rc = tfa2_cnt_write_msg(tfa, size, prof->list[i].offset+2+(char*)tfa->cnt);
				break;
			default:
				/* This allows us to write bitfield, registers or xmem after files */
				rc = tfa2_cnt_write_item(tfa, &prof->list[i]);
				if ( rc < 0 )
					return rc;
				break;
		}

		if ( rc < 0 )
			return rc;

	}
	/*
	 * not in any or same group and not on internal clock
	 */
	if ((prof->group != previous_prof->group || prof->group == 0) ) {
		rc = tfa2_dev_set_state(tfa, TFA_STATE_POWERUP);
	}

	return rc;
}

/*
 * lookup slave and return device index
 */
int tfa2_cnt_get_idx(struct tfa2_device *tfa)
{
	nxpTfaDeviceList_t *dev = NULL;
	int i;

	for (i=0; i<tfa->cnt->ndev; i++) {
		dev = tfa2_cnt_device(tfa->cnt, i);
		if (dev->dev == tfa->slave_address)
			break;

	}
	if (i == tfa->cnt->ndev)
		return -1;

	return i;
}

