import pymysql

# MySQL 데이터베이스 연결 설정
DB_CONFIG = {
    'host': 'localhost',
    'user': 'root',
    'password': 'your_password',  # MySQL 비밀번호 입력
    'database': 'RestaurantSystem'  # 데이터베이스 이름
}


def create_tables():
    try:
        connection = pymysql.connect(
            host=DB_CONFIG['host'],
            user=DB_CONFIG['user'],
            password=DB_CONFIG['password'],
            database=DB_CONFIG['database']
        )
        cursor = connection.cursor()

        # 테이블 생성 쿼리
        table_queries = [
            """
            CREATE TABLE IF NOT EXISTS Menu (
                menu_id INT PRIMARY KEY AUTO_INCREMENT,
                menu_name VARCHAR(255) NOT NULL,
                price INT NOT NULL,
                cooking_time INT NOT NULL,
                image_path VARCHAR(255)
            );
            """,
            """
            CREATE TABLE IF NOT EXISTS Cancel (
                cancel_id INT PRIMARY KEY AUTO_INCREMENT,
                reason VARCHAR(255) NOT NULL
            );
            """,
            """
            CREATE TABLE IF NOT EXISTS Review (
                customer_id INT PRIMARY KEY,
                rating INT NOT NULL CHECK (rating BETWEEN 1 AND 5),
                review_comment TEXT
            );
            """,
            """
            CREATE TABLE IF NOT EXISTS SalesRecords (
                customer_id INT NOT NULL,
                menu_id INT
            );
            """,
            """
            CREATE TABLE IF NOT EXISTS Orders (
                order_id INT PRIMARY KEY,
                customer_id INT NOT NULL,
                menu_id INT NOT NULL,
                quantity INT NOT NULL,
                order_time DATETIME NOT NULL,
                cancel_id INT,
                FOREIGN KEY (menu_id) REFERENCES Menu(menu_id),
                FOREIGN KEY (cancel_id) REFERENCES Cancel(cancel_id)
            );
            """,
        ]

        # 쿼리 실행
        for query in table_queries:
            cursor.execute(query)
            print("Table created successfully.")

        connection.commit()
        print("All tables created successfully.")
    except pymysql.MySQLError as e:
        print(f"Error: {e}")
    finally:
        if connection:
            connection.close()
            print("Connection closed.")


# 메뉴 데이터를 MySQL에 저장하는 함수
def save_menu_data_to_db(menu_data):
    """
    menu_data 딕셔너리를 Menu 테이블에 삽입합니다.

    :param menu_data: 메뉴 정보를 담은 딕셔너리
    """
    try:
        # MySQL 데이터베이스 연결
        connection = pymysql.connect(**DB_CONFIG)
        cursor = connection.cursor()

        # SQL 쿼리: 데이터 삽입
        insert_query = """
            INSERT INTO Menu (menu_id, menu_name, price, cooking_time)
            VALUES (%s, %s, %s, %s)
            ON DUPLICATE KEY UPDATE
                menu_name = VALUES(menu_name),
                price = VALUES(price),
                cooking_time = VALUES(cooking_time);
        """

        # 데이터를 하나씩 삽입
        for menu_id, menu_info in menu_data.items():
            cursor.execute(
                insert_query,
                (menu_id, menu_info['menu_name'], menu_info['price'], menu_info['cooking_time'])
            )

        # 변경 사항 저장
        connection.commit()
        print("Menu data successfully saved to the database.")

    except pymysql.MySQLError as e:
        print(f"Error saving menu data to the database: {e}")
    finally:
        if connection:
            connection.close()


def get_menu_image_path(menu_id):
    """
    메뉴 ID를 입력받아 해당 메뉴의 이미지 경로를 반환

    :param menu_id: 메뉴 ID (int)
    :return: 이미지 경로 (str) 또는 None
    """
    try:
        connection = pymysql.connect(**DB_CONFIG)
        cursor = connection.cursor()

        # 메뉴 ID로 이미지 경로 조회
        query = "SELECT image_path FROM Menu WHERE menu_id = %s;"
        cursor.execute(query, (menu_id,))
        result = cursor.fetchone()

        return result[0] if result else None
    except pymysql.MySQLError as e:
        print(f"Error fetching image path: {e}")
        return None
    finally:
        if connection:
            connection.close()

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

    # 데이터 삽입 함수
def fetch_menu_order_summary():
    """
    Orders 테이블에서 각 메뉴의 주문 수량을 집계하여 반환.
    :return: {menu_id: total_quantity} 형태의 딕셔너리
    """
    try:
        connection = pymysql.connect(**DB_CONFIG)
        cursor = connection.cursor()

        query = """
            SELECT menu_id, SUM(quantity) AS total_quantity
            FROM Orders
            GROUP BY menu_id;
        """
        cursor.execute(query)
        results = cursor.fetchall()  # [(menu_id, total_quantity), ...]

        # 딕셔너리 형태로 변환
        order_summary = {menu_id: total_quantity for menu_id, total_quantity in results}

        cursor.close()
        connection.close()
        return order_summary
    except pymysql.MySQLError as e:
        print(f"Error fetching menu order summary: {e}")
        return {}

def fetch_most_ordered_menu_with_name():
    """
    Orders 테이블에서 가장 많이 주문된 메뉴의 이름과 주문 수량을 가져옵니다.
    :return: {'menu_name': menu_name, 'total_quantity': total_quantity} 형태의 딕셔너리
    """
    try:
        connection = pymysql.connect(**DB_CONFIG)
        cursor = connection.cursor()

        # 메뉴 이름 포함 쿼리
        query = """
            SELECT Menu.menu_name, SUM(Orders.quantity) AS total_quantity
            FROM Orders
            JOIN Menu ON Orders.menu_id = Menu.menu_id
            GROUP BY Orders.menu_id
            ORDER BY total_quantity DESC
            LIMIT 1;
        """
        cursor.execute(query)
        result = cursor.fetchone()  # (menu_name, total_quantity)

        cursor.close()
        connection.close()

        if result:
            return {'menu_name': result[0], 'total_quantity': result[1]}
        else:
            return None  # 데이터가 없는 경우

    except pymysql.MySQLError as e:
        print(f"Error fetching most ordered menu with name: {e}")
        return None

def get_menu_image_path(menu_name):
    """
    메뉴 이름에 해당하는 이미지 경로를 반환합니다.
    """
    menu_images = {
        "Tomato Pasta": "tomatopasta.jpeg",
        "Cream Pasta": "creampasta.jpeg",
        "Vongole Pasta": "vongolepasta.jpeg",
    }
    return menu_images.get(menu_name, "default_image.jpeg")

def insert_cancel_reason(reason):
    connection = pymysql.connect(**DB_CONFIG)
    query = "INSERT INTO Cancel (reason) VALUES (%s)"
    cursor = connection.cursor()
    cursor.execute(query, (reason,))
    connection.commit()
    print(f"취소 사유 추가 완료: {reason}")

def insert_null_menu_id(customer_id):
    """
    SalesRecords 테이블에 customer_id와 menu_id=NULL로 데이터를 삽입.
    
    :param customer_id: INT, 고객 ID
    """
    # MySQL 데이터베이스 연결
    connection = pymysql.connect(**DB_CONFIG)
    cursor = connection.cursor()
    # 데이터 삽입
    try:
        cursor.execute(
            """
            INSERT INTO SalesRecords (customer_id, menu_id)
            VALUES (%s, NULL);
            """,
            (customer_id,)
        )
        connection.commit()
        print("Record inserted successfully.")
    except mysql.connector.Error as e:
        print(f"Error occurred: {e}")
    finally:
        # 연결 종료
        cursor.close()
        connection.close()


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
def insert_order(order_id, customer_id, menu_id, quantity, order_time, cancel_id=None):
    """Orders 테이블에 데이터를 삽입"""
    try:
        connection = pymysql.connect(**DB_CONFIG)
        cursor = connection.cursor()

        query = """
            INSERT INTO Orders (order_id, customer_id, menu_id, quantity, order_time, cancel_id)
            VALUES (%s, %s, %s, %s, %s, %s)
        """
        cursor.execute(query, (order_id, customer_id, menu_id, quantity, order_time, cancel_id))
        connection.commit()

        cursor.close()
        connection.close()
    except pymysql.MySQLError as e:
        print(f"Error inserting order: {e}")

def insert_review_to_db(customer_id, rating, review_comment):
    """
    Review 테이블에 데이터를 삽입합니다.
    
    :param rating: 별점 (int)
    :param review_comment: 리뷰 내용 (str)
    """
    try:
        connection = pymysql.connect(**DB_CONFIG)
        cursor = connection.cursor()

        # SQL 쿼리 실행
        query = """
            INSERT INTO Review (customer_id, rating, review_comment)
            VALUES (%s, %s, %s);
        """
        cursor.execute(query, (customer_id, rating, review_comment))
        connection.commit()

        cursor.close()
        connection.close()
    except pymysql.MySQLError as e:
        raise Exception(f"MySQL Error: {e}")
    
def insert_sales_record(customer_id, menu_id):
    """
    SalesRecords 테이블에 데이터를 삽입하는 함수.

    :param customer_id: 고객 ID (int)
    :param menu_id: 메뉴 ID (int)
    :param menu_item: 메뉴 이름 (str)
    """
    try:
        # 데이터베이스 연결
        connection = pymysql.connect(**DB_CONFIG)
        cursor = connection.cursor()

        # SQL 쿼리: 데이터 삽입
        query = """
            INSERT INTO SalesRecords (customer_id, menu_id)
            VALUES (%s, %s);
        """
        cursor.execute(query, (customer_id, menu_id))

        # 변경 사항 저장
        connection.commit()
        print(f"[INFO] Sales record inserted: customer_id={customer_id}, menu_id={menu_id}")
    except pymysql.MySQLError as e:
        print(f"[ERROR] MySQL Error: {e}")
    finally:
        if connection:
            connection.close()


def update_order_cancel_id(order_id, cancel_id):
    """
    Orders 테이블에서 order_id를 기준으로 cancel_id 값을 수정하는 함수.
    :param order_id: 수정할 주문의 order_id (int, Primary Key)
    :param cancel_id: 새로 설정할 cancel_id (int)
    """
    try:
        connection = pymysql.connect(**DB_CONFIG)
        cursor = connection.cursor()

        query = """
            UPDATE Orders
            SET cancel_id = %s
            WHERE order_id = %s
        """
        cursor.execute(query, (cancel_id, order_id))
        connection.commit()

        print(f"[INFO] order_id={order_id}의 cancel_id가 {cancel_id}로 수정되었습니다.")
    except pymysql.MySQLError as e:
        print(f"[ERROR] 주문 수정 실패: {e}")
    finally:
        cursor.close()
        connection.close()
        
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
    print("Menu cache initialized.")
    return menu_cache, cancel_cache


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


# 5. 메인 함수
if __name__ == "__main__":
    print(fetch_orders())
    create_tables()
    menu_data = {
        1: {'menu_name': 'Tomato Pasta', 'price': 12000, 'cooking_time': 3, 'image_path': 'tomatopasta.jpeg'},
        2: {'menu_name': 'Cream Pasta', 'price': 13000, 'cooking_time': 5, 'image_path': 'creampasta.jpeg'},
        3: {'menu_name': 'Vongole Pasta', 'price': 14000, 'cooking_time': 5, 'image_path': 'vongolepasta.jpeg'},
        4: {'menu_name': 'PePepperoni Pizza', 'price': 15000, 'cooking_time': 4, 'image_path': 'pepperonipizza.jpeg'},
        5: {'menu_name': 'Combination Pizza', 'price': 16000, 'cooking_time': 4, 'image_path': 'combinationpizza.jpeg'},
        6: {'menu_name': 'Hawaiian Pizza', 'price': 15000, 'cooking_time': 3, 'image_path': 'hawaiianpizza.jpeg'},
        7: {'menu_name': 'Tenderloin Steak', 'price': 30000, 'cooking_time': 2, 'image_path': 'tenderloinsteak.jpeg'},
        8: {'menu_name': 'Sirloin Steak', 'price': 28000, 'cooking_time': 2, 'image_path': 'sirloinsteak.jpeg'},
        9: {'menu_name': 'T-bone Steak', 'price': 35000, 'cooking_time': 3, 'image_path': 't-bonesteak.jpeg'},
        10: {'menu_name': 'Coke', 'price': 2000, 'cooking_time': 0, 'image_path': 'coke.jpeg'},
        11: {'menu_name': 'Cider', 'price': 2000, 'cooking_time': 0, 'image_path': 'cider.jpeg'},
        12: {'menu_name': 'BlueBerry Ade', 'price': 4000, 'cooking_time': 1, 'image_path': 'blueberryade.jpeg'}
    }
    save_menu_data_to_db(menu_data)
    insert_cancel_reason('단순 변심')
    insert_cancel_reason('조리 지연')
    insert_cancel_reason('주문 실수')
    insert_cancel_reason('기타 사유')
    insert_cancel_reason('재료 소진')
    print(fetch_menu_as_dict())
    print(fetch_cancel_as_dict())
    print(fetch_orders())
