CREATE TABLE detection (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    date TEXT NOT NULL,
    lat REAL NOT NULL,
    lon REAL NOT NULL,
    image_name TEXT NOT NULL
);

CREATE TABLE tree (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    detection INTEGER NOT NULL,
    color TEXT CHECK( color IN ('R', 'B') ) NOT NULL,

    FOREIGN KEY (detection) REFERENCES detection(id)
);
