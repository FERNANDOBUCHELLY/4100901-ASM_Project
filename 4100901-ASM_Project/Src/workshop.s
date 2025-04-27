// --- Código extendido para detectar pulsación de botón y encender LED --------

    .section .text
    .syntax unified
    .thumb

    .global main
    .global init_led
    .global init_button
    .global init_systick
    .global SysTick_Handler
    .global delay_counter

// --- Definiciones de registros y pines ---------------------------------------
    .equ RCC_BASE,        0x40021000
    .equ RCC_AHB2ENR,     RCC_BASE + 0x4C

    .equ GPIOA_BASE,      0x48000000
    .equ GPIOA_MODER,     GPIOA_BASE + 0x00
    .equ GPIOA_ODR,       GPIOA_BASE + 0x14
    .equ LD2_PIN,         5

    .equ GPIOC_BASE,      0x48000800
    .equ GPIOC_MODER,     GPIOC_BASE + 0x00
    .equ GPIOC_IDR,       GPIOC_BASE + 0x10
    .equ BUTTON_PIN,      13

    .equ SYST_CSR,        0xE000E010
    .equ SYST_RVR,        0xE000E014
    .equ SYST_CVR,        0xE000E018
    .equ HSI_FREQ,        4000000

// --- Variable global para el contador de retardo -----------------------------
    .section .bss
    .align 4
delay_counter:
    .word 0

// --- Programa principal ------------------------------------------------------
    .section .text
main:
    bl init_led
    bl init_button
    bl init_systick

main_loop:
    bl check_button
    ldr r0, =delay_counter
    ldr r1, [r0]
    cmp r1, #0
    beq main_loop               @ Si delay_counter es 0, LED está apagado
    b main_loop

// --- Inicialización de LED PA5 -----------------------------------------------
init_led:
    movw r0, #:lower16:RCC_AHB2ENR
    movt r0, #:upper16:RCC_AHB2ENR
    ldr r1, [r0]
    orr r1, r1, #(1 << 0)       @ Habilita reloj GPIOA
    str r1, [r0]

    movw r0, #:lower16:GPIOA_MODER
    movt r0, #:upper16:GPIOA_MODER
    ldr r1, [r0]
    bic r1, r1, #(0b11 << (LD2_PIN * 2))
    orr r1, r1, #(0b01 << (LD2_PIN * 2)) @ PA5 como salida
    str r1, [r0]
    bx lr

// --- Inicialización del botón PC13 como entrada ------------------------------
init_button:
    movw r0, #:lower16:RCC_AHB2ENR
    movt r0, #:upper16:RCC_AHB2ENR
    ldr r1, [r0]
    orr r1, r1, #(1 << 2)       @ Habilita reloj GPIOC
    str r1, [r0]

    movw r0, #:lower16:GPIOC_MODER
    movt r0, #:upper16:GPIOC_MODER
    ldr r1, [r0]
    bic r1, r1, #(0b11 << (BUTTON_PIN * 2)) @ PC13 como entrada
    str r1, [r0]
    bx lr

// --- Leer botón y encender LED si se presiona -------------------------------
check_button:
    movw r0, #:lower16:GPIOC_IDR
    movt r0, #:upper16:GPIOC_IDR
    ldr r1, [r0]
    tst r1, #(1 << BUTTON_PIN)
    bne check_button_end          @ Si está en alto, no hacer nada

    ldr r0, =delay_counter
    ldr r1, [r0]
    cmp r1, #0
    bne check_button_end          @ Si ya está contando, no hacer nada

    mov r1, #3                   @ 3 segundos
    str r1, [r0]

    // Enciende el LED
    movw r0, #:lower16:GPIOA_ODR
    movt r0, #:upper16:GPIOA_ODR
    ldr r1, [r0]
    orr r1, r1, #(1 << LD2_PIN)
    str r1, [r0]

check_button_end:
    bx lr

// --- Inicialización del SysTick para 1 Hz (1s) ------------------------------
init_systick:
    movw r0, #:lower16:SYST_RVR
    movt r0, #:upper16:SYST_RVR
    movw r1, #:lower16:HSI_FREQ
    movt r1, #:upper16:HSI_FREQ
    subs r1, r1, #1
    str r1, [r0]

    movw r0, #:lower16:SYST_CSR
    movt r0, #:upper16:SYST_CSR
    movs r1, #(1 << 0)|(1 << 1)|(1 << 2)
    str r1, [r0]
    bx lr

// --- Manejador de interrupción de SysTick -----------------------------------
.thumb_func
SysTick_Handler:
    ldr r0, =delay_counter
    ldr r1, [r0]
    cmp r1, #0
    beq systick_end
    subs r1, r1, #1
    str r1, [r0]

    cmp r1, #0
    bne systick_end

    // Apagar LED cuando llegue a 0
    movw r0, #:lower16:GPIOA_ODR
    movt r0, #:upper16:GPIOA_ODR
    ldr r1, [r0]
    bic r1, r1, #(1 << LD2_PIN)
    str r1, [r0]

systick_end:
    bx lr