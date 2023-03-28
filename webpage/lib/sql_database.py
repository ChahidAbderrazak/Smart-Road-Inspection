# Source: https://www.freecodecamp.org/news/connect-python-with-sql/

import mysql.connector
from mysql.connector import Error
import pandas as pd

def create_server_connection(host_name, user_name, user_password):
    '''
    create an SQL connection to access manage the databases
    '''
    connection = None
    try:
        connection = mysql.connector.connect(
            host=host_name,
            user=user_name,
            passwd=user_password
        )
        print("MySQL Database connection successful")
    except Error as err:
        print(f'\nError: {err}')
    return connection

def create_database(connection, DB_name):
    '''
    # create a database
    '''
    mycursor = connection.cursor()
    # mycursor.execute(f"CREATE DATABASE HAIS_DB")
    mycursor.execute(f"CREATE DATABASE {DB_name}")

def show_databases(connection):
    '''
    Show all databases
    '''
    mycursor = connection.cursor()
    mycursor.execute("SHOW DATABASES")
    
    print(f'\n\n -> the SQL databases are:')

    for x in mycursor:
        print(x)

def open_database(host_name, user_name, user_password, database):
    '''
    open a database
    '''
    mydb = None
    try:
        mydb = mysql.connector.connect(
            host=host_name,
            user=user_name,
            passwd=user_password,
            database=database
        )
        print(f"->  {database} Database is opened successfully!")
    except Error as err:
        print(f'\nError: {err}')

    return mydb

def send_query(mydb, sql_query, disp=0):
    '''
    send a sql_query to the open database 
    '''
    try:
        mycursor = mydb.cursor()
        mycursor.execute(sql_query)
        try:
            mydb.commit()
        except:
            pass
        # get the results
        myresult = mycursor.fetchall()
        if disp>=1:
            for x in myresult:
                print(x) 

    except Error as err:
        print(f'\n warning: cannot apply the sent sql_query {sql_query} scheduler table!! \n Error: {err}')
    return myresult

def show_tables(mydb):
    '''
    Show all databases
    '''
    mycursor = mydb.cursor()
    mycursor.execute("SHOW TABLES")
    print(f'\n\n -> the database tables are:')
    for x in mycursor:
        print(x)

#---------------   TABLES  ------------------------
def delete_table(mydb, table):
    '''
    delete a table from the database
    '''
    try:
        sql_query = f"DROP TABLE {table}"
        mycursor = mydb.cursor()
        mycursor.execute(sql_query)
    except Error as err:
        print(f'\n warning: cannot delete the table {table}!! \n Error:{err}')
 

def create_scheduler_table(mydb):
    '''
    scheduler_table=(id, location, date, responsible)
    '''
    try:
        mycursor = mydb.cursor()
        sql_query="CREATE TABLE scheduler (id INT AUTO_INCREMENT PRIMARY KEY, location VARCHAR(255), date VARCHAR(255), responsible VARCHAR(255))"
        mycursor.execute(sql_query)
 
    except Error as err:
        print(f'\n warning: cannot create scheduler table!! \n Error:{err}')
 
    
def update_table(mydb, new_row):
    try:
        mycursor = mydb.cursor()
        sql = "INSERT INTO scheduler (location, date, responsible) VALUES (%s, %s, %s)"
        mycursor.execute(sql, new_row)
        mydb.commit()
        print(f'\n {mycursor.rowcount} successfully record inserted.')
    except Error as err:
        print(f'\n warning: cannot update scheduler table!!\n - new row= {new_row} \n\n Error:{err}')


def test_create_database():
    # input parameters
    host_name="localhost"
    user_name="root"
    user_password='!@Password1000'
    database= "HAIS_DB"

    # create SQL conenction
    connection = create_server_connection(host_name=host_name, 
                                          user_name=user_name, 
                                          user_password=user_password)
    
    # create a database
    create_database(connection, database=database)

    # Show all databases
    show_databases(connection)

def test_database_queries():
    # input parameters
    host_name="localhost"
    user_name="root"
    user_password='!@Password1000'
    database= "HAIS_DB"

    # open an existing database
    mydb=open_database( host_name=host_name, 
                        user_name=user_name, 
                        user_password=user_password,
                        database=database)

    # # delte a table from the database
    # delete_table(mydb, table='scheduler')

    # create scheduler table: (id, location, date, responsible)
    create_scheduler_table(mydb)

    # update the table
    new_row = ("montreal21", "18/12/2022", "Salah")
    update_table(mydb, new_row)

    # send queries: UPDATE
    sql_query = "UPDATE scheduler SET date = '15/15/15' WHERE id = 1"
    send_query(mydb, sql_query, disp=1)

    # send queries: SELECT , DELETE
    sql_query="SELECT * FROM scheduler WHERE id >0 ORDER BY date"
    send_query(mydb, sql_query, disp=1)




 
 



if __name__ == '__main__':
    # # DB creation
    # test_create_database()

    # DB management
    test_database_queries()
 
  