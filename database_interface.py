import pymysql

# MySQL 데이터베이스 연결 설정
DB_CONFIG = {
    'host': 'localhost',
    'user': 'root',
    'password': 'your_password',  # MySQL 비밀번호를 입력하세요
    'database': 'RestaurantSystem'
}


def fetch_menu_as_dict():
    """Menu 테이블에서 데이터를 가져와 딕셔너리로 반환"""
    try:
        connection = pymysql.connect(**DB_CONFIG)
        cursor = connection.cursor()

        query = "SELECT menu_id, menu_name, price, cooking_time FROM Menu"
        cursor.execute(query)
        menu_data = cursor.fetchall()  # [(1, 'Pizza', 15000, 20), ...]

        cursor.close()
        connection.close()

        # 딕셔너리로 변환
        menu_dict = {}
        for item in menu_data:
            menu_id, menu_name, price, cooking_time = item
            menu_dict[menu_id] = {
                'name': menu_name,
                'price': price,
                'cooking_time': cooking_time
            }

        return menu_dict
    except pymysql.MySQLError as e:
        print(f"Error fetching menu from DB: {e}")
        return {}


def fetch_cancel_as_dict():
    """Cancel 테이블에서 데이터를 가져와 딕셔너리로 반환"""
    try:
        connection = pymysql.connect(**DB_CONFIG)
        cursor = connection.cursor()

        query = "SELECT cancel_id, reason FROM Cancel"
        cursor.execute(query)
        cancel_data = cursor.fetchall()  # [(1, 'Customer changed mind'), ...]

        cursor.close()
        connection.close()

        # 딕셔너리로 변환
        cancel_dict = {cancel_id: reason for cancel_id, reason in cancel_data}
        return cancel_dict
    except pymysql.MySQLError as e:
        print(f"Error fetching cancel data from DB: {e}")
        return {}
    
    
# MySQL 유틸리티 함수
def get_max_order_id():
    """Orders 테이블에서 가장 큰 order_id를 조회"""
    try:
        connection = pymysql.connect(**DB_CONFIG)
        cursor = connection.cursor()

        query = "SELECT MAX(order_id) FROM Orders;"
        cursor.execute(query)
        result = cursor.fetchone()
        cursor.close()
        connection.close()

        return result[0] if result[0] is not None else 0

    except pymysql.MySQLError as e:
        print(f"Error fetching max order_id: {e}")
        return None
    
    
    
    
def get_max_customer_id():
    """SalesRecords 테이블에서 가장 큰 customer_id를 조회"""
    try:
        connection = pymysql.connect(**DB_CONFIG)
        cursor = connection.cursor()

        query = "SELECT MAX(customer_id) FROM SalesRecords;"
        cursor.execute(query)
        result = cursor.fetchone()
        cursor.close()
        connection.close()

        return result[0] if result[0] is not None else 0

    except pymysql.MySQLError as e:
        print(f"Error fetching max customer_id: {e}")
        return None


# 2. Orders 테이블에 데이터 삽입
def insert_order(customer_id, order_id, menu_id, quantity, cancel_id=None):
    """Orders 테이블에 데이터를 삽입"""
    try:
        connection = pymysql.connect(**DB_CONFIG)
        cursor = connection.cursor()

        query = """
            INSERT INTO Orders (customer_id, order_id, menu_id, quantity, cancel_id)
            VALUES (%s, %s, %s, %s, %s)
        """
        cursor.execute(query, (customer_id, order_id, menu_id, quantity, cancel_id))
        connection.commit()

        print(f"Order {order_id} inserted successfully.")
        cursor.close()
        connection.close()
    except pymysql.MySQLError as e:
        print(f"Error inserting order: {e}")


# 3. Orders 테이블 데이터 조회
def fetch_orders():
    """Orders 테이블에서 데이터를 조회"""
    try:
        connection = pymysql.connect(**DB_CONFIG)
        cursor = connection.cursor()

        query = "SELECT * FROM Orders"
        cursor.execute(query)
        orders = cursor.fetchall()  # [(customer_id, order_id, menu_id, quantity, cancel_id), ...]

        cursor.close()
        connection.close()
        return orders
    except pymysql.MySQLError as e:
        print(f"Error fetching orders: {e}")
        return []


def initialize_cache():
    """프로그램 시작 시 한 번 메뉴 데이터를 캐싱"""
    menu_cache = fetch_menu_as_dict()
    cancel_cache = fetch_cancel_as_dict()
    return menu_cache, cancel_cache
    print("Menu cache initialized.")


# 4. 메뉴 출력
def display_menu():
    """캐싱된 메뉴 데이터를 출력"""

    print("Menu List:")
    for item in menu_cache:
        print(f"ID: {item[0]}, Name: {item[1]}, Price: {item[2]}, Cooking Time: {item[3]} mins")



# 공통 함수: MySQL 실행
def execute_query(query, params):
    try:
        connection = pymysql.connect(**DB_CONFIG)
        cursor = connection.cursor()
        cursor.execute(query, params)
        connection.commit()
        cursor.close()
        connection.close()
        print("Query executed successfully.")
    except pymysql.MySQLError as e:
        print(f"Error executing query: {e}")


# 1. Menu 테이블에 데이터 삽입
def insert_menu(menu_name, price, cooking_time):
    query = """
        INSERT INTO Menu (menu_name, price, cooking_time)
        VALUES (%s, %s, %s)
    """
    execute_query(query, (menu_name, price, cooking_time))


# 2. Cancel 테이블에 데이터 삽입
def insert_cancel(cancel_id, reason):
    query = """
        INSERT INTO Cancel (cancel_id, reason)
        VALUES (%s, %s)
    """
    execute_query(query, (cancel_id, reason))


# 3. Orders 테이블에 데이터 삽입
def insert_order(customer_id, order_id, menu_id, quantity, cancel_id=None):
    query = """
        INSERT INTO Orders (customer_id, order_id, menu_id, quantity, cancel_id)
        VALUES (%s, %s, %s, %s, %s)
    """
    execute_query(query, (customer_id, order_id, menu_id, quantity, cancel_id))


# 4. Review 테이블에 데이터 삽입
def insert_review(customer_id, rating, review_comment):
    query = """
        INSERT INTO Review (customer_id, rating, review_comment)
        VALUES (%s, %s, %s)
    """
    execute_query(query, (customer_id, rating, review_comment))


# 5. SalesRecords 테이블에 데이터 삽입
def insert_sales_record(customer_id, menu_item):
    query = """
        INSERT INTO SalesRecords (customer_id, menu_item)
        VALUES (%s, %s)
    """
    execute_query(query, (customer_id, menu_item))


# 데이터 삽입 인터페이스
def data_insertion_interface():
    while True:
        print("\nSelect a table to insert data:")
        print("1. Menu")
        print("2. Cancel")
        print("3. Orders")
        print("4. Review")
        print("5. SalesRecords")
        print("6. Exit")

        choice = input("Enter your choice: ").strip()
        if choice == "1":
            menu_name = input("Enter Menu Name: ").strip()
            price = int(input("Enter Price: "))
            cooking_time = int(input("Enter Cooking Time (in mins): "))
            insert_menu(menu_name, price, cooking_time)
        elif choice == "2":
            cancel_id = int(input("Enter Cancel ID: "))
            reason = input("Enter Cancel Reason: ").strip()
            insert_cancel(cancel_id, reason)
        elif choice == "3":
            customer_id = int(input("Enter Customer ID: "))
            order_id = int(input("Enter Order ID: "))
            menu_id = int(input("Enter Menu ID: "))
            quantity = int(input("Enter Quantity: "))
            cancel_id = input("Enter Cancel ID (or press Enter if none): ")
            cancel_id = int(cancel_id) if cancel_id else None
            insert_order(customer_id, order_id, menu_id, quantity, cancel_id)
        elif choice == "4":
            customer_id = int(input("Enter Customer ID: "))
            rating = int(input("Enter Rating (1-5): "))
            review_comment = input("Enter Review Comment: ").strip()
            insert_review(customer_id, rating, review_comment)
        elif choice == "5":
            customer_id = int(input("Enter Customer ID: "))
            menu_item = input("Enter Menu Item: ").strip()
            insert_sales_record(customer_id, menu_item)
        elif choice == "6":
            print("Exiting...")
            break
        else:
            print("Invalid choice. Please try again.")




# 5. 메인 함수
if __name__ == "__main__":
    # 프로그램 실행 시 캐시 초기화
    menu_cache, cancel_cache = initialize_cache()

    while True:
        print("\n1. Display Menu\n2. Insert Data\n3. View Orders\n4. Exit")
        choice = input("Enter your choice: ").strip()

        if choice == "1":
            # 메뉴 출력
            display_menu()
        elif choice == "2":
            # 주문 데이터 삽입
            data_insertion_interface()
        elif choice == "3":
            # 주문 데이터 조회
            orders = fetch_orders()
            if not orders:
                print("No orders found.")
            else:
                print("Order List:")
                for order in orders:
                    print(f"Customer ID: {order[0]}, Order ID: {order[1]}, Menu ID: {order[2]}, Quantity: {order[3]}, Cancel ID: {order[4]}")
        elif choice == "4":
            print("Exiting...")
            break
        else:
            print("Invalid choice. Please try again.")
