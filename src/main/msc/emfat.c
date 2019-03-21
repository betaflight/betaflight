/*
 * Derived from
 * https://github.com/fetisov/emfat/blob/master/project/emfat.c
 * version: 1.1 (2.04.2017)
 */

/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 by Sergey Fetisov <fsenok@gmail.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "platform.h"

#include "common/utils.h"

#include "emfat.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SECT              512
#define CLUST             4096
#define SECT_PER_CLUST    (CLUST / SECT)
#define SIZE_TO_NSECT(s)  ((s) == 0 ? 1 : ((s) + SECT - 1) / SECT)
#define SIZE_TO_NCLUST(s) ((s) == 0 ? 1 : ((s) + CLUST - 1) / CLUST)

#define CLUST_FREE     0x00000000
#define CLUST_RESERVED 0x00000001
#define CLUST_BAD      0x0FFFFFF7
#define CLUST_ROOT_END 0X0FFFFFF8
#define CLUST_EOF      0x0FFFFFFF

#define MAX_DIR_ENTRY_CNT 16
#define FILE_SYS_TYPE_OFF 82
#define BYTES_PER_SEC_OFF 11
#define SEC_PER_CLUS_OFF 13
#define RES_SEC_CNT_OFF 14
#define FAT_CNT_OFF 16
#define TOT_SEC_CNT_OFF 32
#define SEC_PER_FAT 36
#define ROOT_DIR_STRT_CLUS_OFF 44
#define FS_INFOSECTOR_OFF 48
#define BACKUP_BOOT_SEC_OFF 50
#define NXT_FREE_CLUS_OFF 492
#define FILE_SYS_TYPE_LENGTH 8
#define SHRT_FILE_NAME_LEN 11
#define STRT_CLUS_LOW_OFF 26
#define STRT_CLUS_HIGH_OFF 20
#define FILE_SIZE_OFF 28
#define ATTR_OFF 11
#define FILE_STAT_LEN 21
#define CHECK_SUM_OFF 13
#define FILE_NAME_SHRT_LEN 8
#define FILE_NAME_EXTN_LEN 3
#define LONG_FILE_NAME_LEN 255
#define LOW_CLUSWORD_MASK 0x0000FFFF
#define HIGH_CLUSWORD_MASK 0xFFFF0000
#define LONG_FNAME_MASK 0x0F
#define LAST_ORD_FIELD_SEQ 0x40
#define LFN_END_MARK 0xFFFF
#define LFN_TERM_MARK 0x0000
#define LFN_FIRST_OFF 0x01
#define LFN_SIXTH_OFF 0x0E
#define LFN_TWELVETH_OFF 0x1C
#define LFN_FIRST_SET_CNT 5
#define LFN_SEC_SET_CNT 6
#define LFN_THIRD_SET_CNT 2
#define LFN_FIRST_SET_LEN 10
#define LFN_SEC_SET_LEN 12
#define LFN_THIRD_SET_LEN 4
#define LFN_EMPTY_LEN 2
#define LFN_LEN_PER_ENTRY 13
#define FNAME_EXTN_SEP_OFF 6
#define FNAME_SEQ_NUM_OFF 7
#define BYTES_PER_CLUSTER_ENTRY 4
#define DIR_ENTRY_LEN 32
#define VOL_ID_LEN 4
#define VOL_LABEL_LEN 11
#define RESERV_LEN 12
#define FS_VER_LEN 2
#define OEM_NAME_LEN 8
#define JUMP_INS_LEN 3
#define MAX_FAT_CNT 2
#define SPACE_VAL 32
#define FILE_READ 0x01
#define FILE_WRITE 0X02
#define FILE_CREATE_NEW 0x04
#define FILE_CREATE_ALWAYS 0x08
#define FILE_APPEND 0x10
#define FREE_DIR_ENTRY 0x00
#define DEL_DIR_ENTRY 0xE5
#define DOT_DIR_ENTRY 0x2E
#define ASCII_DIFF 32
#define FILE_SEEK_SET 0
#define FILE_SEEK_CUR 1
#define FILE_SEEK_END 2
#define DELIMITER '/'
#define EXTN_DELIMITER '.'
#define TILDE '~'
#define FULL_SHRT_NAME_LEN 13

#pragma pack(push, 1)

typedef struct
{
    uint8_t  status;          // 0x80 for bootable, 0x00 for not bootable, anything else for invalid
    uint8_t  start_head;      // The head of the start
    uint8_t  start_sector;    // (S | ((C >> 2) & 0xC0)) where S is the sector of the start and C is the cylinder of the start. Note that S is counted from one.
    uint8_t  start_cylinder;  // (C & 0xFF) where C is the cylinder of the start
    uint8_t  PartType;
    uint8_t  end_head;
    uint8_t  end_sector;
    uint8_t  end_cylinder;
    uint32_t StartLBA;        // linear address of first sector in partition. Multiply by sector size (usually 512) for real offset
    uint32_t EndLBA;          // linear address of last sector in partition. Multiply by sector size (usually 512) for real offset
} mbr_part_t;

typedef struct
{
    uint8_t    Code[440];
    uint32_t   DiskSig;  //This is optional
    uint16_t   Reserved; //Usually 0x0000
    mbr_part_t PartTable[4];
    uint8_t    BootSignature[2]; //0x55 0xAA for bootable
} mbr_t;

typedef struct
{
    uint8_t jump[JUMP_INS_LEN];
    uint8_t OEM_name[OEM_NAME_LEN];
    uint16_t bytes_per_sec;
    uint8_t sec_per_clus;
    uint16_t reserved_sec_cnt;
    uint8_t fat_cnt;
    uint16_t root_dir_max_cnt;
    uint16_t tot_sectors;
    uint8_t media_desc;
    uint16_t sec_per_fat_fat16;
    uint16_t sec_per_track;
    uint16_t number_of_heads;
    uint32_t hidden_sec_cnt;
    uint32_t tol_sector_cnt;
    uint32_t sectors_per_fat;
    uint16_t ext_flags;
    uint8_t fs_version[FS_VER_LEN];
    uint32_t root_dir_strt_cluster;
    uint16_t fs_info_sector;
    uint16_t backup_boot_sector;
    uint8_t reserved[RESERV_LEN];
    uint8_t drive_number;
    uint8_t reserved1;
    uint8_t boot_sig;
    uint8_t volume_id[VOL_ID_LEN];
    uint8_t volume_label[VOL_LABEL_LEN];
    uint8_t file_system_type[FILE_SYS_TYPE_LENGTH];
} boot_sector;

typedef struct
{
    uint32_t signature1;     /* 0x41615252L */
    uint32_t reserved1[120]; /* Nothing as far as I can tell */
    uint32_t signature2;     /* 0x61417272L */
    uint32_t free_clusters;  /* Free cluster count.  -1 if unknown */
    uint32_t next_cluster;   /* Most recently allocated cluster */
    uint32_t reserved2[3];
    uint32_t signature3;
} fsinfo_t;

typedef struct
{
    uint8_t name[FILE_NAME_SHRT_LEN];
    uint8_t extn[FILE_NAME_EXTN_LEN];
    uint8_t attr;
    uint8_t reserved;
    uint8_t crt_time_tenth;
    uint16_t crt_time;
    uint16_t crt_date;
    uint16_t lst_access_date;
    uint16_t strt_clus_hword;
    uint16_t lst_mod_time;
    uint16_t lst_mod_date;
    uint16_t strt_clus_lword;
    uint32_t size;
} dir_entry;

typedef struct
{
    uint8_t ord_field;
    uint8_t fname0_4[LFN_FIRST_SET_LEN];
    uint8_t flag;
    uint8_t reserved;
    uint8_t chksum;
    uint8_t fname6_11[LFN_SEC_SET_LEN];
    uint8_t empty[LFN_EMPTY_LEN];
    uint8_t fname12_13[LFN_THIRD_SET_LEN];
} lfn_entry;

#pragma pack(pop)

bool emfat_init_entries(emfat_entry_t *entries)
{
    emfat_entry_t *e;
    int i, n;

    e = &entries[0];
    if (e->level != 0 || !e->dir || e->name == NULL) return false;

    e->priv.top = NULL;
    e->priv.next = NULL;
    e->priv.sub = NULL;
    e->priv.num_subentry = 0;

    n = 0;
    for (i = 1; entries[i].name != NULL; i++) {
        entries[i].priv.top = NULL;
        entries[i].priv.next = NULL;
        entries[i].priv.sub = NULL;
        entries[i].priv.num_subentry = 0;
        if (entries[i].level == n - 1) {
            if (n == 0) return false;
            e = e->priv.top;
            n--;
        }

        if (entries[i].level == n + 1) {
            if (!e->dir) return false;
            e->priv.sub = &entries[i];
            entries[i].priv.top = e;
            e = &entries[i];
            n++;
            continue;
        }

        if (entries[i].level == n) {
            if (n == 0) return false;
            e->priv.top->priv.num_subentry++;
            entries[i].priv.top = e->priv.top;
            e->priv.next = &entries[i];
            e = &entries[i];
            continue;
        }

        return false;
    }

    return true;
}

static void lba_to_chs(int lba, uint8_t *cl, uint8_t *ch, uint8_t *dh)
{
    int cylinder, head, sector;
    int sectors = 63;
    int heads = 255;
    int cylinders = 1024;
    sector = lba % sectors + 1;
    head = (lba / sectors) % heads;
    cylinder = lba / (sectors * heads);
    if (cylinder >= cylinders) {
      *cl = *ch = *dh = 0xff;
      return;
    }
    *cl = sector | ((cylinder & 0x300) >> 2);
    *ch = cylinder & 0xFF;
    *dh = head;
}

bool emfat_init(emfat_t *emfat, const char *label, emfat_entry_t *entries)
{
    uint32_t sect_per_fat;
    uint32_t clust;
    uint32_t reserved_clust = 0;
    emfat_entry_t *e;
    int i;

    if (emfat == NULL || label == NULL || entries == NULL) {
        return false;
    }

    if (!emfat_init_entries(entries)) {
        return false;
    }

    clust = 2;
    for (i = 0; entries[i].name != NULL; i++) {
        e = &entries[i];
        if (e->dir) {
            e->curr_size = 0;
            e->max_size = 0;
            e->priv.first_clust = clust;
            e->priv.last_clust = clust + SIZE_TO_NCLUST(e->priv.num_subentry * sizeof(dir_entry)) - 1;
            e->priv.last_reserved = e->priv.last_clust;
        } else {
            e->priv.first_clust = clust;
            e->priv.last_clust = e->priv.first_clust + SIZE_TO_NCLUST(entries[i].curr_size) - 1;
            e->priv.last_reserved = e->priv.first_clust + SIZE_TO_NCLUST(entries[i].max_size) - 1;
        }
        reserved_clust += e->priv.last_reserved - e->priv.last_clust;
        clust = e->priv.last_reserved + 1;
    }
    clust -= 2;

    emfat->vol_label = label;
    emfat->priv.num_entries = i;
    emfat->priv.boot_lba = 62;
    emfat->priv.fsinfo_lba = emfat->priv.boot_lba + 1;
    emfat->priv.fat1_lba = emfat->priv.fsinfo_lba + 1;
    emfat->priv.num_clust = clust;
    emfat->priv.free_clust = reserved_clust;
    sect_per_fat = SIZE_TO_NSECT((uint64_t)emfat->priv.num_clust * 4);
    emfat->priv.fat2_lba = emfat->priv.fat1_lba + sect_per_fat;
    emfat->priv.root_lba = emfat->priv.fat2_lba + sect_per_fat;
    emfat->priv.entries = entries;
    emfat->priv.last_entry = entries;
    emfat->disk_sectors = clust * SECT_PER_CLUST + emfat->priv.root_lba;
    emfat->vol_size = (uint64_t)emfat->disk_sectors * SECT;
    /* calc cyl number */
//    i = ((emfat->disk_sectors + 63*255 - 1) / (63*255));
//    emfat->disk_sectors = i * 63*255;
    return true;
}

void read_mbr_sector(const emfat_t *emfat, uint8_t *sect)
{
    mbr_t *mbr;
    memset(sect, 0, SECT);
    mbr = (mbr_t *)sect;
    mbr->DiskSig = 0;
    mbr->Reserved = 0;
    mbr->PartTable[0].status = 0x80;
    mbr->PartTable[0].PartType = 0x0C;
    mbr->PartTable[0].StartLBA = emfat->priv.boot_lba;
    mbr->PartTable[0].EndLBA = emfat->disk_sectors;
    lba_to_chs(mbr->PartTable[0].StartLBA, &mbr->PartTable[0].start_sector, &mbr->PartTable[0].start_cylinder, &mbr->PartTable[0].start_head);
    lba_to_chs(emfat->disk_sectors - 1, &mbr->PartTable[0].end_sector, &mbr->PartTable[0].end_cylinder, &mbr->PartTable[0].end_head);
    mbr->BootSignature[0] = 0x55;
    mbr->BootSignature[1] = 0xAA;
}

void read_boot_sector(const emfat_t *emfat, uint8_t *sect)
{
    boot_sector *bs;
    memset(sect, 0, SECT);
    bs = (boot_sector *)sect;
    bs->jump[0] = 0xEB;
    bs->jump[1] = 0x58;
    bs->jump[2] = 0x90;
    memcpy(bs->OEM_name, "MSDOS5.0", 8);
    bs->bytes_per_sec = SECT;
    bs->sec_per_clus = 8;     /* 4 kb per cluster */
    bs->reserved_sec_cnt = 2; /* boot sector & fsinfo sector */
    bs->fat_cnt = 2;          /* two tables */
    bs->root_dir_max_cnt = 0;
    bs->tot_sectors = 0;
    bs->media_desc = 0xF8;
    bs->sec_per_fat_fat16 = 0;
    bs->sec_per_track = 63;
    bs->number_of_heads = 0xFF;
    bs->hidden_sec_cnt = 62;
    bs->tol_sector_cnt = emfat->disk_sectors - emfat->priv.boot_lba;
    bs->sectors_per_fat = emfat->priv.fat2_lba - emfat->priv.fat1_lba;
    bs->ext_flags = 0;
    bs->fs_version[0] = 0;
    bs->fs_version[1] = 0;
    bs->root_dir_strt_cluster = 2;
    bs->fs_info_sector = 1;
    bs->backup_boot_sector = 0; /* not used */
    bs->drive_number = 128;
    bs->boot_sig = 0x29;
    bs->volume_id[0] = 148;
    bs->volume_id[1] = 14;
    bs->volume_id[2] = 13;
    bs->volume_id[3] = 8;
    memcpy(bs->volume_label, "NO NAME     ", 12);
    memcpy(bs->file_system_type, "FAT32   ", 8);
    sect[SECT - 2] = 0x55;
    sect[SECT - 1] = 0xAA;
}

#define IS_CLUST_OF(clust, entry) ((clust) >= (entry)->priv.first_clust && (clust) <= (entry)->priv.last_reserved)

emfat_entry_t *find_entry(const emfat_t *emfat, uint32_t clust, emfat_entry_t *nearest)
{
    if (nearest == NULL) {
        nearest = emfat->priv.entries;
    }

    if (nearest->priv.first_clust > clust) {
        while (nearest >= emfat->priv.entries) { // backward finding
            if (IS_CLUST_OF(clust, nearest))
                return nearest;
            nearest--;
        }
    } else {
        while (nearest->name != NULL) { // forward finding
            if (IS_CLUST_OF(clust, nearest))
                return nearest;
            nearest++;
        }
    }
    return NULL;
}

void read_fsinfo_sector(const emfat_t *emfat, uint8_t *sect)
{
    UNUSED(emfat);

    fsinfo_t *info = (fsinfo_t *)sect;
    info->signature1 = 0x41615252L;
    info->signature2 = 0x61417272L;
    //info->free_clusters = 0;
    info->free_clusters = emfat->priv.free_clust;
    //info->next_cluster = emfat->priv.num_clust + 2;
    info->next_cluster = 0xffffffff;
    memset(info->reserved1, 0, sizeof(info->reserved1));
    memset(info->reserved2, 0, sizeof(info->reserved2));
    info->signature3 = 0xAA550000;
}

void read_fat_sector(emfat_t *emfat, uint8_t *sect, uint32_t index)
{
    emfat_entry_t *le;
    uint32_t *values;
    uint32_t count;
    uint32_t curr;

    values = (uint32_t *)sect;
    curr = index * 128;
    count = 128;

    if (curr == 0) {
        *values++ = CLUST_ROOT_END;
        *values++ = 0xFFFFFFFF;
        count -= 2;
        curr += 2;
    }

    le = emfat->priv.last_entry;
    while (count != 0) {
        if (!IS_CLUST_OF(curr, le)) {
            le = find_entry(emfat, curr, le);
            if (le == NULL) {
                le = emfat->priv.last_entry;
                *values = CLUST_RESERVED;
                values++;
                count--;
                curr++;
                continue;
            }
        }
        if (le->dir) {
            if (curr == le->priv.last_clust) {
                *values = CLUST_EOF;
            } else {
                *values = curr + 1;
            }
        } else {
            if (curr == le->priv.last_clust) {
                *values = CLUST_EOF;
            } else if (curr > le->priv.last_clust) {
                *values = CLUST_FREE;
            } else {
                *values = curr + 1;
            }
        }
        values++;
        count--;
        curr++;
    }
    emfat->priv.last_entry = le;
}

void fill_entry(dir_entry *entry, const char *name, uint8_t attr, uint32_t clust, const uint32_t cma[3], uint32_t size)
{
    int i, l, l1, l2;
    int dot_pos;

    memset(entry, 0, sizeof(dir_entry));

    if (cma) {
        entry->crt_date = cma[0] >> 16;
        entry->crt_time = cma[0] & 0xFFFF;
        entry->lst_mod_date = cma[1] >> 16;
        entry->lst_mod_time = cma[1] & 0xFFFF;
        entry->lst_access_date = cma[2] >> 16;
    }

    l = strlen(name);
    dot_pos = -1;

    if ((attr & ATTR_DIR) == 0) {
        for (i = l - 1; i >= 0; i--) {
            if (name[i] == '.')
            {
                dot_pos = i;
                break;
            }
        }
    }

    if (dot_pos == -1) {
        l1 = l > FILE_NAME_SHRT_LEN ? FILE_NAME_SHRT_LEN : l;
        l2 = 0;
    } else {
        l1 = dot_pos;
        l1 = l1 > FILE_NAME_SHRT_LEN ? FILE_NAME_SHRT_LEN : l1;
        l2 = l - dot_pos - 1;
        l2 = l2 > FILE_NAME_EXTN_LEN ? FILE_NAME_EXTN_LEN : l2;
    }

    memset(entry->name, ' ', FILE_NAME_SHRT_LEN + FILE_NAME_EXTN_LEN);
    memcpy(entry->name, name, l1);
    memcpy(entry->extn, name + dot_pos + 1, l2);

    for (i = 0; i < FILE_NAME_SHRT_LEN; i++) {
        if (entry->name[i] >= 'a' && entry->name[i] <= 'z') {
            entry->name[i] -= 0x20;
        }
    }

    for (i = 0; i < FILE_NAME_EXTN_LEN; i++) {
        if (entry->extn[i] >= 'a' && entry->extn[i] <= 'z') {
            entry->extn[i] -= 0x20;
        }
    }

    entry->attr = attr;
    entry->reserved = 24;
    entry->strt_clus_hword = clust >> 16;
    entry->strt_clus_lword = clust;
    entry->size = size;

    return;
}

void fill_dir_sector(emfat_t *emfat, uint8_t *data, emfat_entry_t *entry, uint32_t rel_sect)
{
    dir_entry *de;
    uint32_t avail;

    memset(data, 0, SECT);
    de = (dir_entry *)data;
    avail = SECT;

    if (rel_sect == 0) { // 1. first sector of directory
        if (entry->priv.top == NULL) {
            fill_entry(de++, emfat->vol_label, ATTR_VOL_LABEL, 0, 0, 0);
            avail -= sizeof(dir_entry);
        } else {
            fill_entry(de++, ".", ATTR_DIR | ATTR_READ, entry->priv.first_clust, 0, 0);
            if (entry->priv.top->priv.top == NULL) {
                fill_entry(de++, "..", ATTR_DIR | ATTR_READ, 0, 0, 0);
            } else {
                fill_entry(de++, "..", ATTR_DIR | ATTR_READ, entry->priv.top->priv.first_clust, 0, 0);
            }
            avail -= sizeof(dir_entry) * 2;
        }
        entry = entry->priv.sub;
    } else { // 2. not a first sector
        int n;
        n = rel_sect * (SECT / sizeof(dir_entry));
        n -= entry->priv.top == NULL ? 1 : 2;
        entry = entry->priv.sub;

        while (n > 0 && entry != NULL) {
            entry = entry->priv.next;
            n--;
        }
    }

    while (entry != NULL && avail >= sizeof(dir_entry)) {
        if (entry->dir) {
            fill_entry(de++, entry->name, ATTR_DIR | ATTR_READ, entry->priv.first_clust, entry->cma_time, 0);
        } else {
            //fill_entry(de++, entry->name, ATTR_ARCHIVE | ATTR_READ, entry->priv.first_clust, entry->cma_time, entry->curr_size);
            fill_entry(de++, entry->name, ATTR_ARCHIVE | ATTR_READ | entry->attr, entry->priv.first_clust, entry->cma_time, entry->curr_size);
        }
        entry = entry->priv.next;
        avail -= sizeof(dir_entry);
    }
}

void read_data_sector(emfat_t *emfat, uint8_t *data, uint32_t rel_sect)
{
    emfat_entry_t *le;
    uint32_t cluster;
    cluster = rel_sect / 8 + 2;
    rel_sect = rel_sect % 8;

    le = emfat->priv.last_entry;
    if (!IS_CLUST_OF(cluster, le)) {
        le = find_entry(emfat, cluster, le);
        if (le == NULL) {
            int i;
            for (i = 0; i < SECT / 4; i++)
                ((uint32_t *)data)[i] = 0xEFBEADDE;
            return;
        }
        emfat->priv.last_entry = le;
    }

    if (le->dir) {
        fill_dir_sector(emfat, data, le, rel_sect);
        return;
    }

    if (le->readcb == NULL) {
        memset(data, 0, SECT);
    } else {
        uint32_t offset = cluster - le->priv.first_clust;
        offset = offset * CLUST + rel_sect * SECT;
        le->readcb(data, SECT, offset + le->offset, le);
    }

    return;
}

void emfat_read(emfat_t *emfat, uint8_t *data, uint32_t sector, int num_sectors)
{
    while (num_sectors > 0) {
        if (sector >= emfat->priv.root_lba) {
            read_data_sector(emfat, data, sector - emfat->priv.root_lba);
        } else if (sector == 0) {
            read_mbr_sector(emfat, data);
        } else if (sector == emfat->priv.fsinfo_lba) {
            read_fsinfo_sector(emfat, data);
        } else if (sector == emfat->priv.boot_lba) {
            read_boot_sector(emfat, data);
        } else if (sector >= emfat->priv.fat1_lba && sector < emfat->priv.fat2_lba) {
            read_fat_sector(emfat, data, sector - emfat->priv.fat1_lba);
        } else if (sector >= emfat->priv.fat2_lba && sector < emfat->priv.root_lba) {
            read_fat_sector(emfat, data, sector - emfat->priv.fat2_lba);
        } else {
            memset(data, 0, SECT);
        }
        data += SECT;
        num_sectors--;
        sector++;
    }
}

void write_data_sector(emfat_t *emfat, const uint8_t *data, uint32_t rel_sect)
{
    emfat_entry_t *le;
    uint32_t cluster;
    cluster = rel_sect / 8 + 2;
    rel_sect = rel_sect % 8;

    le = emfat->priv.last_entry;

    if (!IS_CLUST_OF(cluster, le)) {
        le = find_entry(emfat, cluster, le);
        if (le == NULL) return;
        emfat->priv.last_entry = le;
    }

    if (le->dir) {
        // TODO: handle changing a filesize
        return;
    }

    if (le->writecb != NULL) {
        le->writecb(data, SECT, rel_sect * SECT + le->offset, le);
    }
}

#define FEBRUARY        2
#define    STARTOFTIME        1970
#define SECDAY            86400L
#define SECYR            (SECDAY * 365)
#define    leapyear(year)        ((year) % 4 == 0)
#define    days_in_year(a)        (leapyear(a) ? 366 : 365)
#define    days_in_month(a)    (month_days[(a) - 1])

static int month_days[12] = {
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

uint32_t emfat_cma_time_from_unix(uint32_t tim)
{
    register int i;
    register long tmp, day;
    int ymd[3];
    int hms[3];

    day = tim / SECDAY;
    tmp = tim % SECDAY;

    /* Hours, minutes, seconds are easy */

    hms[0] = tmp / 3600;
    hms[1] = (tmp % 3600) / 60;
    hms[2] = (tmp % 3600) % 60;

    /* Number of years in days */
    for (i = STARTOFTIME; day >= days_in_year(i); i++)
        day -= days_in_year(i);
    ymd[0] = i;

    /* Number of months in days left */
    if (leapyear(ymd[0])) {
        days_in_month(FEBRUARY) = 29;
    }
    for (i = 1; day >= days_in_month(i); i++) {
        day -= days_in_month(i);
    }
    days_in_month(FEBRUARY) = 28;
    ymd[1] = i;

    /* Days are what is left over (+1) from all that. */
    ymd[2] = day + 1;
    
    return EMFAT_ENCODE_CMA_TIME(ymd[2], ymd[1], ymd[0], hms[0], hms[1], hms[2]);
}

#ifdef __cplusplus
}
#endif
