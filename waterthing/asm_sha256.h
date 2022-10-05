//library copy-pasted here: https://perso.uclouvain.be/fstandae/source_codes/hash_atmel/source/SHA256.asm
//some things adjusted
//.EQU    DATA_NUM_BYTE = 16

static byte hash_state[32] = {0};
static byte hash_mem[41] = {0};

asm volatile (R"()"
              :
              : "h_state" (hash_state*), "h_mem", (hash_mem*)
              : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15", "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23", "r24", "r25", "cc", "memory");

void sha_init() {
  asm volatile (R"()"
                :
                : "h_state" (hash_state*), "h_mem", (hash_mem*)
                : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15", "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23", "r24", "r25", "cc", "memory");

}

void sha_update_16(byte*) { //pointer to 16byte val
  asm volatile (R"()"
                :
                : "h_state" (hash_state*), "h_mem", (hash_mem*)
                : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15", "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23", "r24", "r25", "cc", "memory");

}

void sha_final_32(byte*) { //pointer to 32byte loc to write
  asm("ldi  r24,32");
  asm("ldi  r25,0");

  asm volatile (R"()"
                :
                : "h_state" (hash_state*), "h_mem", (hash_mem*)
                : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15", "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23", "r24", "r25", "cc", "memory");


}
