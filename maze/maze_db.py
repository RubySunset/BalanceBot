from PIL import Image
from datetime import datetime
from pymongo import MongoClient

class MazeDB:

    def set_c_string(self, c_string):
        self.c_string = c_string
    
    def get_c(self):
        client = MongoClient(self.c_string)
        db = client['maze_history']
        col = db['maze_history']
        return col

    # def add_doc(self, maze_bitmap, shortest_path):
    #     col = self.get_c()
    #     now = datetime.now()
    #     dict = {'_id' : str(now), 'bitmap' : maze_bitmap, 'path' : shortest_path}
    #     col.insert_one(dict)

    def add_doc(self, start, end, edges, shortest_path):
        col = self.get_c()
        now = datetime.now()
        dict = {'_id' : str(now), 'start' : start, 'end' : end, 'edges' : edges, 'path' : shortest_path}
        col.insert_one(dict)

if __name__ == '__main__':

    # Connect to DB.
    maze_db = MazeDB()
    maze_db.set_c_string('mongodb://admin:secret@13.43.40.216:6000/admin?authMechanism=DEFAULT')
    col = maze_db.get_c()

    # Add test doc.
    # maze_db.add_doc([[1, 2, 3], [4, 5, 6], [7, 8, 9]], [(0, 0), (1, 1), (2, 2)])

    # Delete all.
    # col.delete_many({})

    # Print all.
    item_details = col.find()
    for item in item_details:
        print(item['_id'])
        # print(item['start'])
        # print(item['end'])
        # print(item['edges'])
        # print(item['path'])
    
    # path = item['path']
    # bitmap = item['bitmap']
    # image = Image.new('RGB', (len(bitmap), len(bitmap[0])))
    # img_pixels = image.load()
    # for i in range(image.size[0]):
    #     for j in range(image.size[1]):
    #         pixel = bitmap[i][j]
    #         flag = False
    #         colour = None
    #         for k in range(len(path)):
    #             if path[k] == [i, j]:
    #                 foo = int(255 * (k + 1) / len(path))
    #                 colour = (0, foo, foo)
    #                 flag = True
    #                 break
    #         if flag:
    #             pass
    #         elif pixel == 1:
    #             colour = (0, 255, 0)
    #         elif pixel == 2:
    #             colour = (255, 0, 0)
    #         elif pixel == 0:
    #             colour = (0, 0, 0)
    #         elif pixel == 3:
    #             colour = (0, 0, 255)
    #         elif pixel == 4:
    #             colour = (255, 255, 255)
    #         elif pixel == 5:
    #             colour = (128, 128, 128)
    #         else:
    #             raise ValueError('Incorrect pixel value: ' + str(pixel) + ' at ' + str((i, j)) + '.')
    #         img_pixels[i, j] = colour
    # image.show()