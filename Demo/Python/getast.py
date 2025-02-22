import os
import ast

def get_classes_functions_constants(filename):
    with open(filename, "r", encoding="utf-8") as file:
        tree = ast.parse(file.read())

    classes = []
    functions = []
    constants = []

    for node in ast.walk(tree):
        # Найти все классы
        if isinstance(node, ast.ClassDef):
            classes.append(node.name)
        
        # Найти все функции
        elif isinstance(node, ast.FunctionDef):
            functions.append(node.name)
        
        # Найти все константы (переменные в верхнем регистре)
        elif isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    # Константой считаем переменные в верхнем регистре
                    if target.id.isupper():
                        constants.append(target.id)
    
    return classes, functions, constants

# Получаем путь к директории, где находится текущий скрипт
script_dir = os.path.dirname(os.path.abspath(__file__))

# Путь к файлу mvsdk.py относительно текущего скрипта
filename = os.path.join(script_dir, "mvsdk.py")

# Проверяем, существует ли файл
if not os.path.exists(filename):
    print(f"Файл '{filename}' не найден.")
else:
    classes, functions, constants = get_classes_functions_constants(filename)

    print("Классы:")
    print(classes)
    print("\nФункции:")
    print(functions)
    print("\nКонстанты:")
    print(constants)
