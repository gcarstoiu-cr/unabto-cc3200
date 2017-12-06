// Nabto includes
#include <unabto/unabto_aes_cbc.h>
#include <unabto/unabto_hmac_sha256.h>
#include <unabto/util/unabto_buffer.h>
#include <unabto/unabto_util.h>

// Standard includes
#include <stdlib.h>

// TI-drivers includes
#include <ti/drivers/crypto/CryptoCC32XX.h>

// Project specific includes
#include "Board.h"

// Driverlib includes
#include "aes.h"


static CryptoCC32XX_Handle          cryptoHandle;
static CryptoCC32XX_HmacMethod      config;
static CryptoCC32XX_HmacParams      hmacParams;
static CryptoCC32XX_EncryptParams   encryptParams; 

static bool firstRun = true;

static uint8_t key[512];
static uint8_t hashResult[CryptoCC32XX_SHA256_DIGEST_SIZE];

static void init_crypto_engine(bool *isFirstRun)
{
    if (*isFirstRun)
    {
        CryptoCC32XX_init();
        CryptoCC32XX_HmacParams_init(&hmacParams);
        cryptoHandle = CryptoCC32XX_open(Board_CRYPTO0, CryptoCC32XX_HMAC|CryptoCC32XX_AES);
        if (cryptoHandle == NULL)
        {
            return;
        }

        *isFirstRun = false;
    }
}

static bool aes128_cbc_crypt(const uint8_t *key, uint8_t *input, uint16_t input_len,
                             uint32_t function)
{
    if ((input_len < 16) || (input_len % 16 != 0)) 
    {
        return false;
    }

    bool retVal = false;

    init_crypto_engine(&firstRun);

    // separate iv and data
    uint8_t *iv = input;
    input += 16;
    input_len -= 16;   

    encryptParams.aes.keySize = CryptoCC32XX_AES_KEY_SIZE_128BIT;
    encryptParams.aes.pKey    = key; // desiredKey length should be as the desiredKeySize
    encryptParams.aes.pIV     = (void *)iv;
    uint8_t *output    = input;
    size_t  outputLength = 0;
    int32_t error = CryptoCC32XX_STATUS_ERROR;
    
    if (AES_CFG_DIR_ENCRYPT == function)
    {
        error = CryptoCC32XX_encrypt( cryptoHandle, CryptoCC32XX_AES_CBC,
                                      input, input_len,
                                      output , &outputLength, &encryptParams );
    }
    else if (AES_CFG_DIR_DECRYPT == function)
    {
        error = CryptoCC32XX_decrypt( cryptoHandle, CryptoCC32XX_AES_CBC,
                                      input, input_len,
                                      output , &outputLength, &encryptParams );
    }

    if ((CryptoCC32XX_STATUS_SUCCESS == error) /*&& (outputLength > 0)*/)
    {
        retVal = true;
    }

    return retVal;
}

bool unabto_aes128_cbc_encrypt(const uint8_t *key, uint8_t *input,
                        uint16_t input_len)
{
    return aes128_cbc_crypt(key, input, input_len, AES_CFG_DIR_ENCRYPT);
}

bool unabto_aes128_cbc_decrypt(const uint8_t *key, uint8_t *input,
                        uint16_t input_len)
{
    return aes128_cbc_crypt(key, input, input_len, AES_CFG_DIR_DECRYPT);
}

void unabto_hmac_sha256_buffers(const buffer_t keys[], uint8_t keys_size,
                                const buffer_t messages[],
                                uint8_t messages_size, uint8_t *mac,
                                uint16_t mac_size)
{
    uint8_t i;

    init_crypto_engine(&firstRun);

    // concatenate keys
    uint16_t key_size = 0;
    for (i = 0; i < keys_size; i++) {
        key_size += keys[i].size;
    }
    UNABTO_ASSERT(key_size <= UNABTO_SHA256_BLOCK_LENGTH);

    memset(key, 0, sizeof(key));
    uint8_t *key_ptr = key;
    for (i = 0; i < keys_size; i++) {
        memcpy(key_ptr, keys[i].data, keys[i].size);
        key_ptr += keys[i].size;
    }

    // concatenate messages
    uint16_t message_size = 0;
    for (i = 0; i < messages_size; i++) {
        message_size += messages[i].size;
    }
    uint8_t *message = (uint8_t *)malloc(message_size);
    uint8_t *message_ptr = message;
    for (i = 0; i < messages_size; i++) {
        memcpy(message_ptr, messages[i].data, messages[i].size);
        message_ptr += messages[i].size;
    }

    // configure and generate hash
    hmacParams.pKey = key;
    hmacParams.moreData = 0;
    config = CryptoCC32XX_HMAC_SHA256;
    
    memset(hashResult, 0, sizeof(hashResult));
    int32_t error = CryptoCC32XX_sign(cryptoHandle, config , message, message_size, hashResult, &hmacParams);
    if (!error)
    {
        memcpy((void *)mac, (void *)hashResult, (size_t)mac_size);
    }
    else
    {
        // TO DO: print error message
    }

    free(message);
}
