void I2C2_write_byte( uint16_t DevAddress , uint8_t command , uint8_t value );
void I2C4_write_byte( uint16_t DevAddress , uint8_t command , uint8_t value );
uint8_t I2C2_read_byte( uint16_t DevAddress , uint8_t command );
uint8_t I2C4_read_byte( uint16_t DevAddress , uint8_t command );
void I2C2_write_word( uint16_t DevAddress , uint8_t command , uint16_t value );
void I2C4_write_word( uint16_t DevAddress , uint8_t command , uint16_t value );
uint16_t I2C2_read_word( uint16_t DevAddress , uint8_t command );
uint16_t I2C4_read_word( uint16_t DevAddress , uint8_t command );
void I2C2_write_bytes( uint16_t DevAddress , uint8_t command , uint8_t *data , uint8_t data_size );
void I2C4_write_bytes( uint16_t DevAddress , uint8_t command , uint8_t *data , uint8_t data_size );
uint8_t * I2C2_read_bytes( uint16_t DevAddress , uint8_t command , uint8_t data_size );
uint8_t * I2C4_read_bytes( uint16_t DevAddress , uint8_t command , uint8_t data_size );
void I2C2_read_array( uint16_t DevAddress , uint8_t command , uint8_t data_size , uint8_t *result );
void I2C4_read_array( uint16_t DevAddress , uint8_t command , uint8_t data_size , uint8_t *result );
/*---------------------------------------------------------------------------------------------*/
/* THE SHT35-DIS TEMPERATURE SENSOR HAS UNIQUE I2C PROTOCOL, SO A SMALL MODIFICATION IS NEEDED */
/*---------------------------------------------------------------------------------------------*/
void SENSIRION_I2C_read_array( uint16_t DevAddress , uint8_t *command , uint8_t *result , uint8_t result_size );
uint16_t SENSIRION_I2C_read_word( uint16_t DevAddress , uint8_t *command );
void SENSIRION_I2C_write_general_call_reset( uint8_t *command );
