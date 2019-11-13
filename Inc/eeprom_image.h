typedef enum
{
	IMAGE_APPLICATION,
	IMAGE_A,
	IMAGE_B,
	IMAGE_DEFAULT,

	NUMBER_OF_IMAGES
} IMAGE;

typedef enum
{
	EEPROM_ADDRESS_SIGNATURE,
	EEPROM_PROGRAMMING_STATUS,
	EEPROM_SOURCE_IMAGE,
	EEPROM_FILE_SIZE_APPLICATION,
	EEPROM_FILE_SIZE_IMAGE_A,
	EEPROM_FILE_SIZE_IMAGE_B,
	EEPROM_FILE_SIZE_DEFAULT_IMAGE,
	EEPROM_APPLICATION_VERSION,
	EEPROM_IMAGE_A_VERSION,
	EEPROM_IMAGE_B_VERSION,
	EEPROM_DEFAULT_IMAGE_VERSION,
	EEPROM_PORT_NUMBER
} EEPROM_ADDRESS;

typedef struct
{
	uint8_t application_is_programmed     : 1;  /* bit 0    */
	uint8_t version_a_is_programmed       : 1;  /* bit 1    */
	uint8_t version_b_is_programmed       : 1;  /* bit 2    */
	uint8_t default_version_is_programmed : 1;  /* bit 3    */
	uint8_t spare                         : 4;  /* bits 4-7 */
} PROGRAMMED_STATUS_STRUCT;

typedef union
{
	PROGRAMMED_STATUS_STRUCT programmed_status_bits;
	uint8_t					 programmed_status_byte;
} IMAGE_PROGRAMMED_STATUS;

typedef union
{
	uint8_t  version_array[4];
	uint32_t version_word;
} VERSION;

typedef struct
{
	uint8_t application 	: 1; /* bit 0    */
	uint8_t image_a			: 1; /* bit 1    */
	uint8_t image_b			: 1; /* bit 2    */
	uint8_t default_image	: 1; /* bit 3	 */
	uint8_t spare			: 4; /* bits 4-7 */
} DESTINATION_IMAGE_STRUCT;

typedef union
{
	DESTINATION_IMAGE_STRUCT destination_image_struct;
	uint8_t					 destination_image_byte;
} DESTINATION_IMAGE;

typedef enum
{
	VERSION_A       = 0xAA,
	VERSION_B       = 0xBB,
	DEFAULT_VERSION = 0xCC
} PROGRAM_IMAGE_VERSION;

void eeprom_image_initialization();
void write_to_eeprom( EEPROM_ADDRESS eeprom_address , ... );

uint8_t eeprom_read_byte( EEPROM_ADDRESS eeprom_address );
uint16_t eeprom_read_word( EEPROM_ADDRESS eeprom_address );
uint32_t eeprom_read_int( EEPROM_ADDRESS eeprom_address );

uint32_t eeprom_image_get_image_version(IMAGE image);
uint32_t eeprom_image_get_crc(IMAGE image);
void erase_flash_pages( uint32_t first_page , uint32_t last_page );
void eeprom_image_copy_image_parameters( IMAGE source_image , IMAGE destination_image );
void eeprom_image_clear_image_parameters(IMAGE image);
void eeprom_image_updated_programmed_image_parameters( IMAGE image , uint32_t version , uint32_t size , uint32_t crc );

