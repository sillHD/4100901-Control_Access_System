# Instalación de librerías necesarias
try:
    import numpy as np
    from PIL import Image
except ImportError:
    import os
    os.system("pip install pillow numpy")
    import numpy as np
    from PIL import Image

def convert_png_to_c_array(image_path, output_file, width=128, height=64):
    """
    Convierte una imagen PNG en un array de C en orden de filas (ROW-MAJOR)
    compatible con la función ssd1306_DrawBitmap(). Cada byte representa 8 píxeles horizontales.
    """
    # Cargar la imagen y convertirla a blanco y negro (1-bit)
    img = Image.open(image_path).convert('1')
    img = img.resize((width, height))  # Redimensionar si es necesario

    img_data = np.array(img)

    # Determinar el ancho en bytes (cada byte representa 8 píxeles)
    byte_width = (width + 7) // 8
    byte_array = [0] * (byte_width * height)

    for y in range(height):
        for x in range(width):
            byte_index = y * byte_width + (x // 8)  # Índice del byte en el array
            bit_position = 7 - (x % 8)  # Posición del bit (MSB primero)

            if img_data[y, x] == 0:  # Píxel negro (activo en la pantalla OLED)
                byte_array[byte_index] |= (1 << bit_position)

    # Generar el código en formato C
    c_code = f"const uint8_t locked[{len(byte_array)}] = {{\n"
    for i, byte in enumerate(byte_array):
        c_code += f" 0x{byte:02X},"
        if (i + 1) % 16 == 0:  # Salto de línea cada 16 bytes
            c_code += "\n"
    c_code += "\n};\n"

    # Guardar el código en un archivo .h
    with open(output_file, 'w') as f:
        f.write(c_code)

    print(f"C array guardado en {output_file}")

def convert_png_to_c_array2(image_path, output_file, width=128, height=64):
    """
    Convierte una imagen PNG en un array de C en orden de filas (ROW-MAJOR)
    compatible con la función ssd1306_DrawBitmap(). Cada byte representa 8 píxeles horizontales.
    """
    # Cargar la imagen y convertirla a blanco y negro (1-bit)
    img = Image.open(image_path).convert('1')
    img = img.resize((width, height))  # Redimensionar si es necesario

    img_data = np.array(img)

    # Determinar el ancho en bytes (cada byte representa 8 píxeles)
    byte_width = (width + 7) // 8
    byte_array = [0] * (byte_width * height)

    for y in range(height):
        for x in range(width):
            byte_index = y * byte_width + (x // 8)  # Índice del byte en el array
            bit_position = 7 - (x % 8)  # Posición del bit (MSB primero)

            if img_data[y, x] == 0:  # Píxel negro (activo en la pantalla OLED)
                byte_array[byte_index] |= (1 << bit_position)

    # Generar el código en formato C
    c_code = f"const uint8_t unlocked[{len(byte_array)}] = {{\n"  
    for i, byte in enumerate(byte_array):
        c_code += f" 0x{byte:02X},"
        if (i + 1) % 16 == 0:  # Salto de línea cada 16 bytes
            c_code += "\n"
    c_code += "\n};\n"

    # Guardar el código en un archivo .h
    with open(output_file, 'w') as f:
        f.write(c_code)

    print(f"C array guardado en {output_file}")

# Uso del script
convert_png_to_c_array("Assets/locked.png","Assets/locked.h",width=128, height=64)

convert_png_to_c_array2("Assets/unlocked.png","Assets/unlocked.h",width=128, height=64)