/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32 VS Code Extension
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
    .section .text            @ Código ejecutable
    .syntax unified           @ Sintaxis unificada ARM/Thumb
    .thumb                    @ Genera instrucciones Thumb (16/32 bits)
    .global main              @ Hacer visible la etiqueta main

    .equ MEM_LOC, 0x20000010  @ Dirección fija en SRAM para datos
    .equ ID, 0x11227861        @ Valor a decrementar
    .equ DATE, 0x1017         @ Fecha de nacimiento

main:
    @ Inicializar registros
    movw    r0, #:lower16:MEM_LOC    @ Parte baja de la dirección
    movt    r0, #:upper16:MEM_LOC    @ Parte alta (ahora R0 = MEM_LOC)
    ldr     r1, =ID                  @ Cargar ID en r1
    str     r1, [r0]                 @ Almacena valor en MEM_LOC

    @ Llamar a la subrutina decrement
    bl      decrement

    @ Bucle infinito de final
loop:
    b       loop


decrement:
    @ r0 apunta a MEM_LOC, obtener valor
    ldr     r1, [r0]           @ Cargar valor de MEM_LOC en r1
    ldr     r2, =DATE          @ Cargar fecha en r2
    subs    r1, r1, r2         @ r1 = r1 - r2
    str     r1, [r0]           @ guarda r1 en MEM_LOC
    cmp     r1, #0             @ ¿ r1 ≤ 0?
    ble     exit_decrement     @ Si sí, salir

    @ Iteración válida
    add     r7, r7, #1         @ Contador en r7
    b       decrement          @ Repetir

exit_decrement:
    @ Retorno a main
    bx      lr